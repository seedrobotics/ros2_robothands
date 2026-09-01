#!/usr/bin/env python3
"""End to end: drive the Gazebo simulation through the driver's native tick
interface and check that the physics actually follows.

    python3 test_sim_driver_interface.py [mimic|independent] [right|left]

Needs ros_gz_sim and gz_ros2_control; skips cleanly when they are absent.
"""
import sys

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState, Range
from seed_hand_msgs.msg import (AllJoints, AllMainBoards, JointListSetSpeedPos,
                                JointSetSpeedPos)

from rh8d_test_support import (Checks, Probe, Simulation, main, require_gazebo,
                               use_test_domain)

use_test_domain(3)

COUPLING = sys.argv[1] if len(sys.argv) > 1 else 'mimic'
SIDE = sys.argv[2] if len(sys.argv) > 2 else 'right'
FINGERS = ('thumb', 'index', 'middle', 'ring', 'little')


class Probe_(Probe):
    def __init__(self, side):
        super().__init__('sim_probe')
        self.jp = 'l_' if side == 'left' else 'r_'
        self.tp = 'L_' if side == 'left' else 'R_'
        self.motors = None
        self.board = None
        self.states = {}
        self.range = None
        self.wrenches = {}
        self.create_subscription(
            AllJoints, self.tp + 'Joints',
            lambda m: setattr(self, 'motors', {j.name: j for j in m.joints}), 10)
        self.create_subscription(
            AllMainBoards, self.tp + 'Main_Boards',
            lambda m: setattr(self, 'board', m.boards[0] if m.boards else None), 10)
        self.create_subscription(
            JointState, 'joint_states',
            lambda m: self.states.update(zip(m.name, m.position)), 50)
        self.create_subscription(
            Range, f'/rh8d/{side}/palm_ir/range',
            lambda m: setattr(self, 'range', m), 10)
        for finger in FINGERS:
            self.create_subscription(
                WrenchStamped, f'/rh8d/{side}/fingertip/{finger}/wrench',
                (lambda f: lambda m: self.wrenches.__setitem__(f, m))(finger), 10)
        self.pub = self.create_publisher(
            JointListSetSpeedPos, self.tp + 'speed_position', 10)

    def command(self, targets, speed=0):
        """targets: {axis name without prefix (or a bus ID string): ticks}"""
        msg = JointListSetSpeedPos()
        for name, ticks in targets.items():
            entry = JointSetSpeedPos()
            entry.name = name if name.isnumeric() else self.jp + name
            entry.target_pos = ticks
            entry.target_speed = speed
            msg.joints.append(entry)
        self.pub.publish(msg)

    def joint(self, name):
        return self.states.get(self.jp + name, 0.0)

    def ticks(self, axis):
        return self.motors[self.jp + axis].present_position


def run():
    require_gazebo()
    check = Checks(f'Gazebo end to end - {SIDE} hand, finger_coupling:={COUPLING}')
    with Simulation(side=SIDE, coupling=COUPLING) as sim:
        p = Probe_(SIDE)
        try:
            ready = p.wait_until(
                lambda: p.motors is not None and len(p.states) > 10, 120,
                'the simulation and driver interface to come up')
            if not check(ready, 'simulation and driver interface come up'):
                sim.dump_log()
                return check.report()
            p.spin(3.0)

            check(len(p.motors) == 8, f'{len(p.motors)} motor axes on {p.tp}Joints')
            check(p.board is not None and p.board.palm_ir_sensor == 255,
                  f'palm IR reads 255 in an empty world '
                  f'({p.board.palm_ir_sensor if p.board else "no message"})')
            check(p.range is not None, 'palm_ir_adapter publishes a Range')
            check(len(p.wrenches) == 5,
                  f'{len(p.wrenches)}/5 fingertip wrench topics bridged')

            check.section('closing a finger from ticks')
            before = p.joint('index_proximal_joint')
            p.command({'index_flexion_joint': 4095})
            closed = p.wait_until(
                lambda: p.joint('index_proximal_joint') > 1.3, 25, 'the index to close')
            p.spin(3.0)                    # settle before reading steady state
            check(closed, f'tick 4095 closes the index in physics '
                          f'({before:.3f} -> {p.joint("index_proximal_joint"):.3f} rad)')
            check(p.joint('index_medial_joint') > 1.4
                  and p.joint('index_distal_joint') > 0.85,
                  f'the whole chain follows (medial '
                  f'{p.joint("index_medial_joint"):.3f}, distal '
                  f'{p.joint("index_distal_joint"):.3f})')
            check(p.ticks('index_flexion_joint') > 3900,
                  f'present_position tracks back to ~4095 '
                  f'({p.ticks("index_flexion_joint")})')
            check(p.motors[p.jp + 'index_flexion_joint'].target_position == 4095,
                  'target_position is echoed')

            p.command({'index_flexion_joint': 0})
            opened = p.wait_until(
                lambda: p.joint('index_proximal_joint') < 0.15, 25, 'the index to open')
            p.spin(2.0)
            check(opened, f'tick 0 reopens the index '
                          f'({p.joint("index_proximal_joint"):.3f} rad)')
            check(p.ticks('index_flexion_joint') < 150,
                  f'present_position returns to ~0 ({p.ticks("index_flexion_joint")})')

            check.section('a direct radian axis')
            target_rad = -1.5708 + 3500 / 4095 * (2 * 1.5708)
            p.command({'wrist_rotation_joint': 3500})
            rotated = p.wait_until(
                lambda: abs(p.joint('wrist_rotation_joint') - target_rad) < 0.02,
                25, 'the wrist to rotate')
            p.spin(1.5)
            check(rotated, f'wrist tick 3500 -> '
                           f'{p.joint("wrist_rotation_joint"):.3f} rad '
                           f'(expect {target_rad:.3f})')
            check(abs(p.ticks('wrist_rotation_joint') - 3500) < 150,
                  f'and reads back as {p.ticks("wrist_rotation_joint")} ticks')

            check.section('bus-ID addressing and the speed profile')
            middle_id = str(47 if SIDE == 'left' else 37)
            p.command({middle_id: 4095}, speed=150)
            p.spin(1.5)
            # measured on the motor axis: in independent mode the coupling
            # node engages the phalanges sequentially, so the proximal joint
            # saturates in the first third of motor travel
            partial = p.ticks('middle_flexion_joint')
            check(200 < partial < 3200,
                  f'bus ID "{middle_id}" with target_speed 150 closes gradually '
                  f'({partial} ticks after 1.5 s, expect ~1800 of 4095)')
            finished = p.wait_until(
                lambda: p.joint('middle_proximal_joint') > 1.3, 30,
                'the slow move to finish')
            check(finished, f'and completes '
                            f'({p.joint("middle_proximal_joint"):.3f} rad)')

            check.section('all eight axes at once')
            # The thumb is kept out of the way: closed across the fingers it
            # now collides with them, so this stays a free-motion check.
            p.command({'wrist_rotation_joint': 2048, 'wrist_flexion_joint': 2048,
                       'wrist_adduction_joint': 2048, 'thumb_adduction_joint': 0,
                       'thumb_flexion_joint': 0, 'index_flexion_joint': 4095,
                       'middle_flexion_joint': 4095, 'ring_little_flexion_joint': 4095})
            curled = ('index', 'middle', 'ring', 'little')
            fist = p.wait_until(
                lambda: all(p.joint(f + '_proximal_joint') > 0.6 for f in curled),
                40, 'the four fingers to curl')
            p.spin(2.0)
            check(fist, 'one command curls all four fingers: '
                        + ', '.join(f'{f}={p.joint(f + "_proximal_joint"):.2f}'
                                    for f in curled))
            currents = {n.replace(p.jp, ''): j.current for n, j in p.motors.items()}
            check.info(f'modelled currents: {currents}')
            check(all(abs(c) < 400 for c in currents.values()),
                  f'modelled current stays low in free motion '
                  f'(max {max(abs(c) for c in currents.values())} mA)')

            check.section('self-collision')
            # Closing the thumb across the curled fingers is a contact now: it
            # must stall short of where it reaches in free space, and the
            # blocked axes must show that in the modelled current.
            p.command({'thumb_adduction_joint': 4095, 'thumb_flexion_joint': 4095})
            p.spin(12.0)
            thumb = p.joint('thumb_proximal_joint')
            blocked = max(abs(j.current) for n, j in p.motors.items()
                          if 'thumb' in n or 'index' in n)
            check.info(f'thumb proximal {thumb:.2f} rad against the fingers, '
                       f'peak current on the blocked axes {blocked} mA')
            check(thumb < 0.70,
                  f'the curled fingers stop the thumb short of the 0.77 rad it '
                  f'reaches in free space ({thumb:.2f})')
            check(blocked > 400,
                  f'and the contact shows up as motor current ({blocked} mA)')
            check(set(j.moving for j in p.motors.values()) <= {0, 1},
                  'the moving flag is boolean')
        finally:
            p.destroy_node()
    return check.report()


if __name__ == '__main__':
    main(run)
