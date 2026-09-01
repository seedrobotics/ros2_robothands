#!/usr/bin/env python3
"""The seed_hand_driver user samples, unmodified, against the simulation.

That is the whole point of driver_interface.py, so it is tested by running
the shipped scripts themselves rather than a re-implementation of them.
Sample 7 needs something to grab, so a static ball is spawned on the palm
IR ray first.
"""
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import tf2_ros
from sensor_msgs.msg import JointState
from seed_hand_msgs.msg import AllJoints, AllMainBoards

from rh8d_test_support import (Checks, Probe, Simulation, main, require_gazebo,
                               terminate, use_test_domain)

use_test_domain(5)

SIDE = 'right'
JP = 'r_'
FINGERS = ('thumb', 'index', 'middle', 'ring', 'little')

BALL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="grab_target">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><sphere><radius>0.025</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.025</radius></sphere></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


class Probe_(Probe):
    def __init__(self):
        super().__init__('user_sample_probe')
        self.motors = None
        self.palm_ir = None
        self.states = {}
        self.create_subscription(
            AllJoints, 'R_Joints',
            lambda m: setattr(self, 'motors', {j.name: j for j in m.joints}), 10)
        self.create_subscription(
            AllMainBoards, 'R_Main_Boards',
            lambda m: setattr(self, 'palm_ir',
                              m.boards[0].palm_ir_sensor if m.boards else None), 10)
        self.create_subscription(
            JointState, 'joint_states',
            lambda m: self.states.update(zip(m.name, m.position)), 50)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def joint(self, name):
        return self.states.get(JP + name, 0.0)

    def palm_ir_pose(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                return self.buffer.lookup_transform('world', JP + 'palm_ir',
                                                    rclpy.time.Time())
            except Exception:
                self.spin(0.2)
        return None


def samples_dir():
    share = subprocess.run(['ros2', 'pkg', 'prefix', '--share', 'seed_hand_driver'],
                           capture_output=True, text=True).stdout.strip()
    return os.path.join(share, 'user_samples')


def run_sample(name, seconds):
    """Run a sample script exactly as a user would, return its output."""
    proc = subprocess.Popen([sys.executable, os.path.join(samples_dir(), name)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    time.sleep(seconds)
    return terminate(proc)


def spawn_ball(transform, distance=0.038):
    """Put a static ball on the palm IR ray, `distance` m from the sensor."""
    t, q = transform.transform.translation, transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    # first column of the rotation matrix - the sensor's +x, its ray direction
    ray = (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w))
    path = os.path.join(tempfile.mkdtemp(prefix='rh8d_grab_'), 'ball.sdf')
    with open(path, 'w') as f:
        f.write(BALL_SDF)
    subprocess.run(
        ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'rh8d_world',
         '-file', path, '-name', 'grab_target',
         '-x', str(t.x + ray[0] * distance),
         '-y', str(t.y + ray[1] * distance),
         '-z', str(t.z + ray[2] * distance)],
        capture_output=True, timeout=90)


def run():
    require_gazebo()
    check = Checks('the seed_hand_driver user samples, against the simulation')
    with Simulation(side=SIDE, coupling='independent') as sim:
        p = Probe_()
        try:
            ready = p.wait_until(
                lambda: p.motors is not None and len(p.states) > 10, 120,
                'the simulation to come up')
            if not check(ready, 'simulation comes up'):
                sim.dump_log()
                return check.report()
            p.spin(3.0)

            check.section('user_sample_1_get_values.py')
            output = run_sample('user_sample_1_get_values.py', 6)
            check('r_index_flexion_joint' in output,
                  'reads the sim\'s joint telemetry through R_Joints')
            check('Number of joints: 8' in output,
                  'sees all 8 motor axes')

            check.section('user_sample_2_set_speed_position_R.py')
            output = run_sample('user_sample_2_set_speed_position_R.py', 6)
            p.spin(8.0)
            check('Published speed/position command' in output,
                  'runs without error')
            expected = -1.5708 + 1000 / 4095 * (2 * 1.5708)   # the sample's tick 1000
            check(abs(p.joint('wrist_rotation_joint') - expected) < 0.05,
                  f'moved the simulated wrist to its commanded tick '
                  f'({p.joint("wrist_rotation_joint"):+.3f} rad, '
                  f'expect {expected:+.3f})')
            check(abs(p.motors['r_wrist_rotation_joint'].present_position - 1000) < 60,
                  f'and R_Joints reports the tick back '
                  f'({p.motors["r_wrist_rotation_joint"].present_position})')

            check.section('user_sample_7_RH8D_R_grab_object.py')
            transform = p.palm_ir_pose()
            if not check(transform is not None, 'the palm IR frame is on TF'):
                return check.report()
            # Start the sample first, let it settle, and only then present
            # the object - the way a person would. The sample publishes its
            # first command the instant the IR trips, and a command published
            # before the subscriber is matched is simply dropped
            # (user_sample_2 sleeps 1 s for exactly this reason). Spawning the
            # ball first would race that discovery and make this test flaky.
            proc = subprocess.Popen(
                [sys.executable, os.path.join(samples_dir(),
                                              'user_sample_7_RH8D_R_grab_object.py')],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                start_new_session=True)
            p.spin(4.0)
            spawn_ball(transform)
            p.spin(3.0)
            check(p.palm_ir is not None and p.palm_ir < 20,
                  f'the palm IR sees the object ({p.palm_ir} mm; the sample '
                  'triggers below 20)')

            # watched while it runs: the sample releases the object at the end
            # and leaves the hand open again
            peak_closure = {f: 0.0 for f in FINGERS}
            peak_current = 0
            end = time.time() + 50
            while time.time() < end and proc.poll() is None:
                p.spin(0.2)
                for finger in FINGERS:
                    peak_closure[finger] = max(peak_closure[finger],
                                               p.joint(finger + '_proximal_joint'))
                if p.motors:
                    peak_current = max(peak_current,
                                       max(j.current for j in p.motors.values()))
            output = terminate(proc)

            check.info('peak closure during the grab: '
                       + ', '.join(f'{k}={v:.2f}' for k, v in peak_closure.items())
                       + f'; highest current seen by this test {peak_current} mA')
            check(any(v > 0.2 for v in peak_closure.values()),
                  'the sample closed the fingers on the object')
            # What matters is what the sample itself saw: it logs the joints
            # whose current passed its 300 mA threshold, and only releases
            # once three of them have. Polling R_Joints from here samples too
            # coarsely to catch the peak reliably.
            stalled = max((entry.count("'r_") for entry in re.findall(
                r"\[('r_[a-z_]+_joint'(?:, 'r_[a-z_]+_joint')*)\]", output)),
                default=0)
            check(stalled >= 3,
                  f'the modelled current made the sample flag {stalled} stalled '
                  'joints (it releases at 3)')
            check('target_speed=10' in output,
                  'the sample ran its state machine through to the release step')
        finally:
            p.destroy_node()
    return check.report()


if __name__ == '__main__':
    main(run)
