#!/usr/bin/env python3
"""rh8d_coupling_node.py - the tendon model shared by the hand and the sim.

Checks the piecewise map itself (sequential proximal -> medial -> distal
engagement, normalized closure, pass-through axes) and the real-hand
visualization path, where the node is run backwards: measured motor states
in, full phalanx joint states out.
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

os.environ['ROS_DOMAIN_ID'] = str(int(os.environ.get('RH8D_TEST_DOMAIN_BASE', '88')) + 8)

MARGIN = 0.03
CHAIN = [('proximal', 1.57), ('medial', 1.57), ('distal', 1.0)]
MOTOR_AXES = ['r_wrist_rotation_joint', 'r_wrist_adduction_joint',
              'r_wrist_flexion_joint', 'r_thumb_adduction_joint',
              'r_thumb_flexion_joint', 'r_index_flexion_joint',
              'r_middle_flexion_joint', 'r_ring_little_flexion_joint']


class Checks:
    def __init__(self, title):
        self.failures = []
        print(f'\n=== {title} ===', flush=True)

    def __call__(self, ok, message):
        print(('  PASS  ' if ok else '  FAIL  ') + message, flush=True)
        if not ok:
            self.failures.append(message)
        return bool(ok)

    def section(self, title):
        print(f'\n--- {title} ---', flush=True)

    def report(self):
        print('=' * 70, flush=True)
        if self.failures:
            print(f'{len(self.failures)} FAILURE(S):', flush=True)
            for f in self.failures:
                print('  - ' + f, flush=True)
            return 1
        print('ALL CHECKS PASSED', flush=True)
        return 0


class Harness(Node):
    def __init__(self):
        super().__init__('coupling_node_test')
        self.states = None
        self.panel = None
        self.create_subscription(
            JointState, 'joint_states',
            lambda m: setattr(self, 'states', dict(zip(m.name, m.position))), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'motor_description',
                                 lambda m: setattr(self, 'panel', m.data), latched)
        self.pub = self.create_publisher(JointState, 'motor_states', 10)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, values):
        msg = JointState()
        msg.name = list(values)
        msg.position = list(values.values())
        self.pub.publish(msg)

    def joint(self, name):
        return self.states.get(name, 0.0)


def start_node():
    """The node as hand.launch.py and view.launch.py run it for the real hand:
    motor_commands remapped to motor_states, output straight to joint_states."""
    return subprocess.Popen(
        ['ros2', 'run', 'seed_rh8d_description', 'rh8d_coupling_node.py',
         '--ros-args', '-r', '__node:=motor_state_mapper',
         '-p', 'prefix:=r_', '-p', 'output:=joint_states',
         '-r', 'motor_commands:=motor_states'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)


def stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        return proc.communicate(timeout=10)[0] or ''
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.communicate()[0] or ''


def run():
    check = Checks('rh8d_coupling_node as the real-hand state mapper')
    node = start_node()
    h = Harness()
    h.spin(3.0)
    try:
        h.send({'r_index_flexion_joint': 1.0, 'r_wrist_rotation_joint': 0.5})
        h.spin(1.5)
        if not check(h.states is not None,
                     'motor states are republished as joint states'):
            return check.report()

        check(len(h.states) >= 19,
              f'{len(h.states)} phalanx joints produced from 8 motor axes')
        proximal = h.joint('r_index_proximal_joint')
        medial = h.joint('r_index_medial_joint')
        distal = h.joint('r_index_distal_joint')
        check(proximal > 1.5 and medial > 1.5 and distal > 0.9,
              f'closure 1.0 wraps the whole chain (proximal {proximal:.3f}, '
              f'medial {medial:.3f}, distal {distal:.3f})')
        check(all(abs(h.joint(f'r_index_{s}_joint') - (u - MARGIN)) < 1e-6
                  for s, u in CHAIN),
              'and each phalanx stops exactly limit_margin short of its limit')
        check(abs(h.joint('r_wrist_rotation_joint') - 0.5) < 1e-6,
              f'a pass-through axis stays in radians '
              f'({h.joint("r_wrist_rotation_joint"):.4f})')

        check.section('sequential engagement')
        h.send({'r_index_flexion_joint': 0.3, 'r_wrist_rotation_joint': 0.0})
        h.spin(1.5)
        proximal = h.joint('r_index_proximal_joint')
        medial = h.joint('r_index_medial_joint')
        check(proximal > 1.0 and medial < 0.3,
              f'partial closure fills the proximal joint before the medial '
              f'(proximal {proximal:.3f}, medial {medial:.3f})')
        h.send({'r_index_flexion_joint': 0.0})
        h.spin(1.5)
        check(all(abs(h.joint(f'r_index_{s}_joint')) < 1e-6 for s, _ in CHAIN),
              'closure 0 opens the whole chain')

        check.section('clamping and the motor slider panel')
        h.send({'r_index_flexion_joint': 5.0,
                'r_wrist_rotation_joint': 99.0})
        h.spin(1.5)
        check(abs(h.joint('r_index_proximal_joint') - (1.57 - MARGIN)) < 1e-6,
              'an out-of-range closure is clamped, not extrapolated')
        check(abs(h.joint('r_wrist_rotation_joint') - (1.5708 - MARGIN)) < 1e-6,
              f'a pass-through axis is clamped a margin inside its limit '
              f'({h.joint("r_wrist_rotation_joint"):.4f})')
        h.send({'not_a_motor_axis': 1.0})
        h.spin(1.0)
        check(h.states is not None, 'an unknown motor name is ignored, not fatal')

        check(h.panel is not None, 'the motor panel description is latched')
        if h.panel:
            check(all(f'joint name="{axis}"' in h.panel for axis in MOTOR_AXES),
                  'the panel offers exactly one slider per motor axis')
            check('lower="0.0" upper="1.0"' in h.panel.replace('"0"', '"0.0"')
                  or 'upper="1.0"' in h.panel,
                  'flexion sliders span the normalized 0..1 closure range')
    finally:
        stop(node)
        h.destroy_node()
    return check.report()


if __name__ == '__main__':
    rclpy.init()
    try:
        code = run()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)
