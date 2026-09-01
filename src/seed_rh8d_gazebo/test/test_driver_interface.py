#!/usr/bin/env python3
"""driver_interface.py on its own - no Gazebo, no hardware.

joint_states is faked, so the feedback path is fully controlled and every
tick conversion, addressing mode, speed profile and validation rule can be
checked exactly. The simulation itself is covered by test_sim_*.py.
"""
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Range
from trajectory_msgs.msg import JointTrajectory
from seed_hand_msgs.msg import (AllJoints, AllMainBoards, ClearHWError,
                                JointListSetSpeedPos, JointListSetStiffness,
                                JointSetSpeedPos, SetShutdownCond, SetStiffness)

from rh8d_test_support import Checks, Probe, main, ros_run, terminate, use_test_domain

use_test_domain(1)

# the model geometry the node assumes; kept here so a change to either side
# of the contract shows up as a failure rather than as silent drift
MARGIN = 0.03
CHAINS = {
    'thumb': [('proximal', 0.8), ('medial', 1.57), ('distal', 0.4)],
    'index': [('proximal', 1.57), ('medial', 1.57), ('distal', 1.0)],
    'middle': [('proximal', 1.57), ('medial', 1.57), ('distal', 1.0)],
    'ring': [('proximal', 1.57), ('medial', 1.57), ('distal', 1.0)],
    'little': [('proximal', 1.57), ('medial', 1.57), ('distal', 1.0)],
}
DIRECT = ['wrist_rotation_joint', 'wrist_adduction_joint',
          'wrist_flexion_joint', 'thumb_adduction_joint']
MOTOR_AXES = DIRECT + ['thumb_flexion_joint', 'index_flexion_joint',
                       'middle_flexion_joint', 'ring_little_flexion_joint']


class Harness(Probe):
    """Stands in for the simulation: publishes joint_states, sends commands."""

    def __init__(self, output_topic):
        super().__init__('driver_interface_test')
        self.joints = []
        self.boards = []
        self.commands = []
        self.create_subscription(AllJoints, 'R_Joints',
                                 self.joints.append, 10)
        self.create_subscription(AllMainBoards, 'R_Main_Boards',
                                 self.boards.append, 10)
        if output_topic == 'trajectory':
            self.create_subscription(JointTrajectory,
                                     'hand_controller/joint_trajectory',
                                     self.commands.append, 10)
        else:
            self.create_subscription(JointState, 'motor_commands',
                                     self.commands.append, 10)
        self.pub_speed_pos = self.create_publisher(
            JointListSetSpeedPos, 'R_speed_position', 10)
        self.pub_stiffness = self.create_publisher(
            JointListSetStiffness, 'R_stiffness', 10)
        self.pub_clear = self.create_publisher(ClearHWError, 'R_clear_error', 10)
        self.pub_shutdown = self.create_publisher(
            SetShutdownCond, 'R_shutdown_condition', 10)
        self.pub_ir = self.create_publisher(Range, 'palm_ir/range', 10)
        self.pub_states = self.create_publisher(JointState, 'joint_states', 10)

        self.measured = {}
        for finger, chain in CHAINS.items():
            for suffix, _ in chain:
                self.measured[f'r_{finger}_{suffix}_joint'] = 0.0
        for axis in DIRECT:
            self.measured['r_' + axis] = 0.0
        self.create_timer(0.02, self._publish_states)

    def _publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.measured)
        msg.position = [self.measured[n] for n in msg.name]
        msg.velocity = [0.0] * len(msg.name)
        self.pub_states.publish(msg)

    def set_closure(self, finger, closure):
        """Pose a finger's phalanx chain at a closure fraction, as the
        coupling node would: proximal consumes travel first."""
        remaining = closure * sum(u - MARGIN for _, u in CHAINS[finger])
        for suffix, upper in CHAINS[finger]:
            value = min(max(remaining, 0.0), upper - MARGIN)
            self.measured[f'r_{finger}_{suffix}_joint'] = value
            remaining -= value

    def command(self, name, position, speed=0):
        msg = JointListSetSpeedPos()
        entry = JointSetSpeedPos()
        entry.name = name
        entry.target_pos = position
        entry.target_speed = speed
        msg.joints.append(entry)
        self.pub_speed_pos.publish(msg)

    def set_stiffness(self, name, value):
        msg = JointListSetStiffness()
        msg.joints.append(SetStiffness(name=name, stiffness=value))
        self.pub_stiffness.publish(msg)

    def motor(self, name):
        return {j.name: j for j in self.joints[-1].joints}[name]


def start_node(**params):
    settings = {'topic_prefix': 'R_', 'joint_prefix': 'r_', 'base_id': 30}
    settings.update(params)
    return ros_run('seed_rh8d_gazebo', 'driver_interface.py', params=settings)


def traj_value(msg, joint):
    return msg.points[-1].positions[list(msg.joint_names).index(joint)]


def state_value(msg, joint):
    return msg.position[list(msg.name).index(joint)]


# ── mimic mode: the node drives the trajectory controller's leader joints ───

def test_mimic(check):
    check.section('mimic mode - trajectory output')
    node = start_node(finger_coupling='mimic')
    h = Harness('trajectory')
    h.spin(2.5)
    try:
        check(len(h.joints) > 20, f'{len(h.joints)} R_Joints messages received')
        check(len(h.boards) > 20, f'{len(h.boards)} R_Main_Boards messages received')
        check(not h.commands,
              f'silent until the first command ({len(h.commands)} sent)')

        report = h.joints[-1]
        names = [j.name for j in report.joints]
        ids = [j.bus_id for j in report.joints]
        check(report.length == 8 and len(report.joints) == 8,
              f'8 motor axes reported (length={report.length})')
        check(ids == list(range(31, 39)), f'bus IDs match RH8D_R.yaml: {ids}')
        check(names == ['r_' + a for a in MOTOR_AXES],
              f'motor names match RH8D_R.yaml: {names}')
        board = h.boards[-1].boards[0]
        check(board.name == 'r_main_board' and board.id == 30,
              f'main board reported as {board.name} id={board.id}')
        check(board.palm_ir_sensor == 255,
              f'palm IR defaults to 255 = nothing in range ({board.palm_ir_sensor})')

        check.section('tick conversion')
        h.commands.clear()
        h.command('r_index_flexion_joint', 4095)
        h.spin(0.6)
        check(len(h.commands) > 10,
              f'commands flow once something is commanded ({len(h.commands)})')
        last = h.commands[-1]
        check(len(last.joint_names) == 8,
              f'all 8 motors commanded every cycle ({len(last.joint_names)})')
        check('r_index_proximal_joint' in last.joint_names,
              'the finger is driven through its mimic leader joint')
        value = traj_value(last, 'r_index_proximal_joint')
        check(abs(value - (1.57 - MARGIN)) < 1e-6,
              f'tick 4095 -> leader {value:.4f} rad (expect {1.57 - MARGIN})')

        h.set_closure('index', 1.0)
        h.spin(0.5)
        motor = h.motor('r_index_flexion_joint')
        check(abs(motor.present_position - 4095) <= 2,
              f'closure 1.0 reads back as {motor.present_position} ticks')
        check(motor.target_position == 4095,
              f'target_position echoed ({motor.target_position})')

        h.command('r_index_flexion_joint', 2048)
        h.set_closure('index', 0.5)
        h.spin(0.5)
        check(abs(h.motor('r_index_flexion_joint').present_position - 2048) <= 3,
              f'closure 0.5 reads back as '
              f'{h.motor("r_index_flexion_joint").present_position} ticks')

        check.section('addressing and direct axes')
        h.command('37', 4095)
        h.spin(0.4)
        value = traj_value(h.commands[-1], 'r_middle_proximal_joint')
        check(abs(value - (1.57 - MARGIN)) < 1e-6,
              f'bus ID "37" addresses the middle finger ({value:.4f} rad)')

        h.command('r_wrist_rotation_joint', 0)
        h.spin(0.4)
        value = traj_value(h.commands[-1], 'r_wrist_rotation_joint')
        check(abs(value - (-1.5708 + MARGIN)) < 1e-6,
              f'wrist tick 0 -> {value:.4f} rad, held a margin inside the limit')
        h.command('r_wrist_rotation_joint', 2048)
        h.spin(0.4)
        value = traj_value(h.commands[-1], 'r_wrist_rotation_joint')
        check(abs(value) < 1e-3, f'wrist tick 2048 -> {value:.4f} rad (centre)')

        h.command('no_such_joint', 100)
        h.spin(0.4)

        check.section('speed profile')
        h.command('r_thumb_flexion_joint', 0)
        h.spin(0.4)
        h.command('r_thumb_flexion_joint', 4095, speed=100)
        h.spin(1.0)
        upper = 0.8 - MARGIN
        fraction = traj_value(h.commands[-1], 'r_thumb_proximal_joint') / upper
        check(0.10 < fraction < 0.30,
              f'target_speed 100 rate-limits the move '
              f'(closure {fraction:.3f} after 1 s, expect ~0.20)')
        h.spin(5.0)
        value = traj_value(h.commands[-1], 'r_thumb_proximal_joint')
        check(abs(value - upper) < 1e-3, f'and reaches the target ({value:.4f})')
        h.command('r_thumb_flexion_joint', 0, speed=-1)
        h.spin(1.0)
        fraction = traj_value(h.commands[-1], 'r_thumb_proximal_joint') / upper
        check(0.7 < fraction < 0.92,
              f'target_speed -1 keeps the previous speed '
              f'(closure {fraction:.3f} after 1 s)')

        check.section('telemetry the simulation models')
        check(h.motor('r_wrist_rotation_joint').moving == 0, 'moving is 0 at rest')
        h.pub_ir.publish(Range(range=0.05))
        h.spin(0.4)
        check(h.boards[-1].boards[0].palm_ir_sensor == 50,
              f'palm IR 0.05 m -> {h.boards[-1].boards[0].palm_ir_sensor} mm')
        h.pub_ir.publish(Range(range=0.255))
        h.spin(0.4)
        check(h.boards[-1].boards[0].palm_ir_sensor == 255,
              f'palm IR no-echo -> {h.boards[-1].boards[0].palm_ir_sensor} mm')

        h.command('r_ring_little_flexion_joint', 4095)
        h.set_closure('ring', 0.2)
        h.set_closure('little', 0.2)
        h.spin(0.8)
        motor = h.motor('r_ring_little_flexion_joint')
        check(motor.current > 500,
              f'a blocked finger raises the modelled current ({motor.current} mA)')
        check(motor.stress_level > 0,
              f'stress_level follows the current ({motor.stress_level} %)')
        h.set_closure('ring', 1.0)
        h.set_closure('little', 1.0)
        h.spin(0.8)
        motor = h.motor('r_ring_little_flexion_joint')
        check(abs(motor.current) < 100,
              f'and falls back to ~0 when tracking again ({motor.current} mA)')

        check.section('validation and rate limiting')
        h.set_stiffness('r_index_flexion_joint', 5)
        h.spin(0.5)
        check(h.motor('r_index_flexion_joint').stiffness == 5,
              f'stiffness accepted and echoed '
              f'({h.motor("r_index_flexion_joint").stiffness})')
        h.set_stiffness('r_index_flexion_joint', 7)
        h.spin(0.5)
        check(h.motor('r_index_flexion_joint').stiffness == 5,
              'a second stiffness write inside 30 s is discarded, as on the bus')
        h.set_stiffness('r_middle_flexion_joint', 12)
        h.spin(0.5)
        check(h.motor('r_middle_flexion_joint').stiffness == 8,
              f'out-of-range stiffness rejected, default kept '
              f'({h.motor("r_middle_flexion_joint").stiffness})')

        h.pub_clear.publish(ClearHWError(name='r_index_flexion_joint'))
        h.pub_clear.publish(ClearHWError(name='r_index_flexion_joint'))
        h.pub_clear.publish(ClearHWError(name='no_such_joint'))
        h.pub_shutdown.publish(SetShutdownCond(name='r_index_flexion_joint',
                                               temperature=True, overload=False))
        h.spin(0.8)
    finally:
        output = terminate(node)
        h.destroy_node()

    check('No mapping for joint "no_such_joint"' in output,
          'an unknown joint name is warned about, not fatal')
    check('CLEARING ERROR' in output, 'clear_error is logged')
    check('too frequent' in output, 'the 30 s clear_error rate limit applies')
    check('SETTING SHUTDOWN CONDITIONS' in output, 'shutdown_condition is logged')
    check('out of range [1-9]' in output, 'the stiffness range is validated')


# ── independent mode: the node feeds the coupling node's motor axes ─────────

def test_independent(check):
    check.section('independent mode - motor_commands output')
    node = start_node(finger_coupling='independent')
    h = Harness('motor_commands')
    h.spin(2.0)
    try:
        check(not h.commands, 'silent until the first command')
        h.command('r_index_flexion_joint', 4095)
        h.spin(0.6)
        check(len(h.commands) > 10, f'motor_commands flow ({len(h.commands)})')
        last = h.commands[-1]
        check(list(last.name) == ['r_' + a for a in MOTOR_AXES],
              f'the 8 coupling-node motor axes are commanded: {list(last.name)}')
        check(abs(state_value(last, 'r_index_flexion_joint') - 1.0) < 1e-6,
              f'tick 4095 -> closure '
              f'{state_value(last, "r_index_flexion_joint"):.4f}')
        h.command('r_index_flexion_joint', 2048)
        h.spin(0.4)
        check(abs(state_value(h.commands[-1], 'r_index_flexion_joint') - 0.5) < 2e-3,
              f'tick 2048 -> closure '
              f'{state_value(h.commands[-1], "r_index_flexion_joint"):.4f}')

        value = state_value(h.commands[-1], 'r_wrist_flexion_joint')
        check(abs(value) < 1e-6,
              f'an uncommanded axis echoes its measured value ({value:.4f})')
        h.measured['r_wrist_flexion_joint'] = 0.4
        h.spin(0.5)
        check(abs(state_value(h.commands[-1], 'r_wrist_flexion_joint') - 0.4) < 1e-6,
              'an uncommanded axis follows the joint if something moves it')
        h.command('r_wrist_flexion_joint', 2048)
        h.spin(0.4)
        h.measured['r_wrist_flexion_joint'] = -0.5
        h.spin(0.5)
        check(abs(state_value(h.commands[-1], 'r_wrist_flexion_joint')) < 2e-3,
              'but a commanded axis holds its target against external motion')
    finally:
        terminate(node)
        h.destroy_node()


def test_split_ring_little(check):
    check.section('independent mode with couple_ring_little:=false')
    node = start_node(finger_coupling='independent', couple_ring_little='false')
    h = Harness('motor_commands')
    h.spin(2.0)
    try:
        h.command('r_ring_little_flexion_joint', 4095)
        h.spin(0.6)
        last = h.commands[-1]
        check(len(last.name) == 9, f'9 output axes ({len(last.name)})')
        check('r_ring_flexion_joint' in last.name
              and 'r_little_flexion_joint' in last.name,
              'the single tendon drives both split axes')
        check(abs(state_value(last, 'r_ring_flexion_joint') - 1.0) < 1e-6
              and abs(state_value(last, 'r_little_flexion_joint') - 1.0) < 1e-6,
              'both split axes get the motor value')
    finally:
        terminate(node)
        h.destroy_node()


def test_calibration(check):
    check.section('calibration overrides')
    node = start_node(finger_coupling='independent',
                      **{'calib.index_flexion_joint.tick_min': 3000,
                         'calib.index_flexion_joint.tick_max': 1000})
    h = Harness('motor_commands')
    h.spin(2.0)
    try:
        h.command('r_index_flexion_joint', 3000)
        h.spin(0.5)
        check(abs(state_value(h.commands[-1], 'r_index_flexion_joint')) < 1e-6,
              'an inverted motor: tick_min maps to closure 0')
        h.command('r_index_flexion_joint', 1000)
        h.spin(0.5)
        check(abs(state_value(h.commands[-1], 'r_index_flexion_joint') - 1.0) < 1e-6,
              'an inverted motor: tick_max maps to closure 1')
        h.set_closure('index', 1.0)
        h.spin(0.5)
        check(abs(h.motor('r_index_flexion_joint').present_position - 1000) <= 2,
              f'and the feedback inverts to match '
              f'({h.motor("r_index_flexion_joint").present_position} ticks)')
    finally:
        terminate(node)
        h.destroy_node()


# ── publish rate ────────────────────────────────────────────────────────────

class Counter(Probe):
    def __init__(self):
        super().__init__('rate_counter')
        self.joints = 0
        self.boards = 0
        self.create_subscription(AllJoints, 'R_Joints',
                                 lambda _: self._bump('joints'), 200)
        self.create_subscription(AllMainBoards, 'R_Main_Boards',
                                 lambda _: self._bump('boards'), 200)

    def _bump(self, field):
        setattr(self, field, getattr(self, field) + 1)


def test_rate(check):
    check.section('publish rate')
    for frequency in (50.0, 20.0):
        node = start_node(frequency=frequency)
        counter = Counter()
        executor = MultiThreadedExecutor()
        executor.add_node(counter)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            time.sleep(3.0)                    # let discovery settle
            counter.joints = counter.boards = 0
            start = time.time()
            time.sleep(5.0)
            elapsed = time.time() - start
            joints_hz = counter.joints / elapsed
            boards_hz = counter.boards / elapsed
            # A generous band: the point is that the timer runs at the
            # configured rate rather than some fixed default, not that a
            # Python timer keeps hard real time on a loaded machine.
            check(abs(joints_hz - frequency) < frequency * 0.2
                  and abs(boards_hz - frequency) < frequency * 0.2,
                  f'frequency:={frequency:.0f} -> R_Joints {joints_hz:.1f} Hz, '
                  f'R_Main_Boards {boards_hz:.1f} Hz')
        finally:
            executor.shutdown()
            counter.destroy_node()
            terminate(node)


def run():
    check = Checks('driver_interface.py, standalone')
    test_mimic(check)
    test_independent(check)
    test_split_ring_little(check)
    test_calibration(check)
    test_rate(check)
    return check.report()


if __name__ == '__main__':
    main(run)
