#!/usr/bin/env python3
"""The two bridge nodes are mirrors of each other, so chaining them is an
identity:

    motor_commands --[aligned_interface]--> R_speed_position (ticks)
                   --[driver_interface]--> motor_commands_out

Any disagreement about a unit, a range or a calibration parameter shows up
here as a value that does not come back unchanged. That is the "one
calibration block serves both interfaces" claim, tested.

Also covers aligned_interface's own inputs: a trajectory of phalanx joint
names, and driver state converted back into aligned units.
"""
import sys

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from seed_hand_msgs.msg import AllJoints, JointListSetSpeedPos, LoneJoint

from rh8d_test_support import Checks, Probe, main, ros_run, terminate, use_test_domain

use_test_domain(2)

MARGIN = 0.03
# axis -> (minimum, maximum) in its aligned unit
AXES = {
    'wrist_rotation_joint': (-1.5708, 1.5708),
    'wrist_adduction_joint': (-0.7854, 0.7854),
    'wrist_flexion_joint': (-0.7854, 0.7854),
    'thumb_adduction_joint': (-0.6, 0.7854),
    'thumb_flexion_joint': (0.0, 1.0),
    'index_flexion_joint': (0.0, 1.0),
    'middle_flexion_joint': (0.0, 1.0),
    'ring_little_flexion_joint': (0.0, 1.0),
}
RADIAN_AXES = {a for a, (lo, hi) in AXES.items() if (lo, hi) != (0.0, 1.0)}


class Harness(Probe):
    def __init__(self):
        super().__init__('parity_test')
        self.returned = None
        self.ticks = None
        self.states = None
        self.create_subscription(
            JointState, 'motor_commands_out',
            lambda m: setattr(self, 'returned', dict(zip(m.name, m.position))), 10)
        self.create_subscription(
            JointListSetSpeedPos, 'R_speed_position',
            lambda m: setattr(self, 'ticks',
                              {j.name: j.target_pos for j in m.joints}), 10)
        self.create_subscription(
            JointState, 'motor_states',
            lambda m: setattr(self, 'states', dict(zip(m.name, m.position))), 10)
        self.pub_commands = self.create_publisher(JointState, 'motor_commands', 10)
        self.pub_trajectory = self.create_publisher(
            JointTrajectory, 'hand_controller/joint_trajectory', 10)
        self.pub_joints = self.create_publisher(AllJoints, 'R_Joints', 10)

    def send(self, values):
        msg = JointState()
        msg.name = ['r_' + a for a in values]
        msg.position = list(values.values())
        self.pub_commands.publish(msg)

    def send_trajectory(self, names, positions):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        msg.points = [JointTrajectoryPoint(
            positions=list(positions),
            time_from_start=Duration(sec=0, nanosec=80_000_000))]
        self.pub_trajectory.publish(msg)


def start_pair(calibration):
    """aligned_interface and driver_interface, both on the same calibration."""
    settings = {'topic_prefix': 'R_', 'joint_prefix': 'r_'}
    aligned = ros_run('seed_hand_driver', 'aligned_interface',
                      params={**settings, **calibration})
    driver = ros_run('seed_rh8d_gazebo', 'driver_interface.py',
                     params={**settings, **calibration,
                             'output': 'motor_commands', 'frequency': 50.0},
                     remaps={'motor_commands': 'motor_commands_out'})
    return aligned, driver


def test_round_trip(check, calibration, label):
    check.section(label)
    aligned, driver = start_pair(calibration)
    h = Harness()
    h.spin(3.5)
    try:
        cases = [
            ('mid-range', {a: (lo + hi) / 2 for a, (lo, hi) in AXES.items()}),
            ('minimum', {a: lo for a, (lo, _) in AXES.items()}),
            ('maximum', {a: hi for a, (_, hi) in AXES.items()}),
            ('mixed', {a: lo + 0.23 * (hi - lo) for a, (lo, hi) in AXES.items()}),
        ]
        for name, values in cases:
            h.returned = None            # so we read this setpoint, not the last
            h.send(values)
            if not h.wait_until(lambda: h.returned is not None, 5.0, 'a round trip'):
                check(False, f'{name}: nothing came back through the pair')
                continue
            h.spin(1.0)                  # let the chain settle on the new value
            worst, worst_axis = 0.0, ''
            for axis, sent in values.items():
                got = h.returned.get('r_' + axis)
                low, high = AXES[axis]
                # the radian axes are deliberately held limit_margin inside
                # their limits (dartsim pins a joint parked on a limit); the
                # flexion axes round-trip exactly
                expected = sent
                if axis in RADIAN_AXES:
                    expected = min(max(sent, low + MARGIN), high - MARGIN)
                error = abs(got - expected) / (high - low)
                if error > worst:
                    worst, worst_axis = error, axis
            check(worst < 0.002,
                  f'{name}: every axis returns within {worst * 100:.3f} % of '
                  f'its range' + (f' (worst {worst_axis})' if worst_axis else ''))
    finally:
        terminate(aligned)
        terminate(driver)
        h.destroy_node()


def test_aligned_inputs(check):
    check.section('aligned_interface inputs and state output')
    aligned = ros_run('seed_hand_driver', 'aligned_interface',
                      params={'topic_prefix': 'R_', 'joint_prefix': 'r_'})
    h = Harness()
    h.spin(3.0)
    try:
        chain = ['r_index_proximal_joint', 'r_index_medial_joint',
                 'r_index_distal_joint']
        h.ticks = None
        h.send_trajectory(chain, [1.57, 1.57, 1.0])
        h.wait_until(lambda: h.ticks is not None, 5.0, 'a tick command')
        check(h.ticks.get('r_index_flexion_joint') == 4095,
              f'a fully closed phalanx trajectory -> '
              f'{h.ticks.get("r_index_flexion_joint")} ticks (expect 4095)')
        h.send_trajectory(chain, [0.785, 0.785, 0.5])
        h.spin(1.0)
        got = h.ticks.get('r_index_flexion_joint')
        check(abs(got - 2048) < 12,
              f'a half closed phalanx trajectory -> {got} ticks (expect ~2048)')
        h.send_trajectory(['r_wrist_flexion_joint'], [0.0])
        h.spin(1.0)
        check(abs(h.ticks.get('r_wrist_flexion_joint') - 2048) < 3,
              f'a motor-axis trajectory name passes straight through '
              f'({h.ticks.get("r_wrist_flexion_joint")} ticks)')

        report = AllJoints()
        for name, ticks in [('r_index_flexion_joint', 4095),
                            ('r_wrist_rotation_joint', 0),
                            ('r_thumb_adduction_joint', 2048)]:
            joint = LoneJoint()
            joint.name = name
            joint.present_position = ticks
            report.joints.append(joint)
        report.length = len(report.joints)
        h.states = None
        h.pub_joints.publish(report)
        h.wait_until(lambda: h.states is not None, 5.0, 'motor_states')
        check(abs(h.states['r_index_flexion_joint'] - 1.0) < 1e-6,
              f'driver state 4095 -> closure {h.states["r_index_flexion_joint"]:.4f}')
        check(abs(h.states['r_wrist_rotation_joint'] + 1.5708) < 1e-4,
              f'driver state 0 -> {h.states["r_wrist_rotation_joint"]:.4f} rad')
        check(abs(h.states['r_thumb_adduction_joint'] - 0.0927) < 1e-3,
              f'driver state 2048 -> {h.states["r_thumb_adduction_joint"]:.4f} rad '
              '(centre of -0.6..0.7854)')
    finally:
        terminate(aligned)
        h.destroy_node()


def run():
    check = Checks('aligned_interface <-> driver_interface parity')
    test_round_trip(check, {}, 'default calibration (0..4095 over the full range)')
    test_round_trip(check, {
        'calib.index_flexion_joint.tick_min': 3200,
        'calib.index_flexion_joint.tick_max': 900,
        'calib.wrist_rotation_joint.tick_min': 400,
        'calib.wrist_rotation_joint.tick_max': 3700,
    }, 'overridden calibration (index inverted, wrist narrowed)')
    test_aligned_inputs(check)
    return check.report()


if __name__ == '__main__':
    main(run)
