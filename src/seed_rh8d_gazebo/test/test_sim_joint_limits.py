#!/usr/bin/env python3
"""Regression: no command may leave a joint stuck.

dartsim pins a joint that reaches its limit - it stays there and ignores
every later command. The coupling node and driver_interface keep commands a
limit_margin inside the limits, but that is not enough on its own: an
undamped joint given a large step overshoots *dynamically* into the limit
and pins there. That is how r_wrist_rotation_joint used to lock up after an
ordinary two-radian step, so the axes are exercised with exactly that
pattern - large steps to mid-range targets, which is where an overshoot has
somewhere to overshoot to.

    python3 test_sim_joint_limits.py [mimic|independent] [right|left]
"""
import sys

from sensor_msgs.msg import JointState
from seed_hand_msgs.msg import AllJoints, JointListSetSpeedPos, JointSetSpeedPos

from rh8d_test_support import (Checks, Probe, Simulation, main, require_gazebo,
                               use_test_domain)

use_test_domain(4)

COUPLING = sys.argv[1] if len(sys.argv) > 1 else 'mimic'
SIDE = sys.argv[2] if len(sys.argv) > 2 else 'right'

DIRECT_AXES = ['wrist_rotation_joint', 'wrist_adduction_joint',
               'wrist_flexion_joint', 'thumb_adduction_joint']
ALL_AXES = DIRECT_AXES + ['thumb_flexion_joint', 'index_flexion_joint',
                          'middle_flexion_joint', 'ring_little_flexion_joint']
# large steps, landing on mid-range targets rather than on the limits
SWEEP = [2500, 3500, 600, 2048, 4000, 100, 3000, 1000]
SETTLE = 4.0
TOLERANCE = 250        # ticks; the margin costs ~40 on the widest axis


class Probe_(Probe):
    def __init__(self, side):
        super().__init__('joint_limit_probe')
        self.jp = 'l_' if side == 'left' else 'r_'
        self.tp = 'L_' if side == 'left' else 'R_'
        self.motors = None
        self.states = {}
        self.create_subscription(
            AllJoints, self.tp + 'Joints',
            lambda m: setattr(self, 'motors', {j.name: j for j in m.joints}), 10)
        self.create_subscription(
            JointState, 'joint_states',
            lambda m: self.states.update(zip(m.name, m.position)), 50)
        self.pub = self.create_publisher(
            JointListSetSpeedPos, self.tp + 'speed_position', 10)

    def command(self, axes, ticks):
        msg = JointListSetSpeedPos()
        for axis in axes:
            entry = JointSetSpeedPos()
            entry.name = self.jp + axis
            entry.target_pos = ticks
            entry.target_speed = 0
            msg.joints.append(entry)
        self.pub.publish(msg)

    def ticks(self, axis):
        return self.motors[self.jp + axis].present_position


def sweep(check, p, axes, targets, label):
    """Command axes through targets, failing on any that does not arrive."""
    stuck = False
    for target in targets:
        p.command(axes, target)
        p.spin(SETTLE)
        missed = {a: p.ticks(a) for a in axes
                  if abs(p.ticks(a) - target) > TOLERANCE}
        if not check(not missed,
                     f'{label}: {target} ticks reached'
                     + (f' - stuck at {missed}' if missed else '')):
            stuck = True
            break          # a pinned joint never recovers; no point continuing
    if not stuck:
        p.command(axes, 2048)
        p.spin(SETTLE + 1.0)
        missed = {a: p.ticks(a) for a in axes
                  if abs(p.ticks(a) - 2048) > TOLERANCE}
        check(not missed,
              f'{label}: still responsive afterwards'
              + (f' - stuck at {missed}' if missed else ''))


def run():
    require_gazebo()
    check = Checks(f'joint limit regression - {SIDE} hand, '
                   f'finger_coupling:={COUPLING}')
    with Simulation(side=SIDE, coupling=COUPLING) as sim:
        p = Probe_(SIDE)
        try:
            ready = p.wait_until(
                lambda: p.motors is not None and len(p.states) > 10, 120,
                'the simulation to come up')
            if not check(ready, 'simulation comes up'):
                sim.dump_log()
                return check.report()
            p.spin(3.0)

            # The historical failure: this axis alone, stepped across its
            # range. It used to fly into -pi/2 on the third target and stay
            # there for good.
            check.section('wrist rotation, driven on its own')
            sweep(check, p, ['wrist_rotation_joint'], SWEEP, 'wrist_rotation')

            check.section('the other direct axes, each on its own')
            for axis in DIRECT_AXES[1:]:
                sweep(check, p, [axis], [3500, 600, 2048], axis.replace('_joint', ''))

            check.section('all eight axes together')
            sweep(check, p, ALL_AXES, SWEEP + [4095, 0], 'all axes')
        finally:
            p.destroy_node()
    return check.report()


if __name__ == '__main__':
    main(run)
