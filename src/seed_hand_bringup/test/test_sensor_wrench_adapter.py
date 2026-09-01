#!/usr/bin/env python3
"""sensor_wrench_adapter.py against a faked FTS sensor stream.

Covers the whole pipeline the node documents - auto tare, axis remap,
deadband, the wrench_display polarity clipping, zeros for silent sensors,
the tare service and live parameter validation - without needing a hand.
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
from ros2_sensor_pkg.msg import AllSensors, LoneSensor

# Own DDS domain: `colcon test` runs packages in parallel, and these topic
# names are the same ones a real hand would be using.
os.environ['ROS_DOMAIN_ID'] = str(int(os.environ.get('RH8D_TEST_DOMAIN_BASE', '88')) + 7)

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']
NODE_NAME = 'sensor_wrench_adapter_right'
REST = (100, 200, -300)          # the fake sensors' unloaded bias


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
        super().__init__('sensor_adapter_test')
        self.wrench = {}
        self.display = {}
        for finger in FINGERS:
            self.create_subscription(
                WrenchStamped, f'/rh8d/right/fingertip/{finger}/wrench',
                (lambda f: lambda m: self.wrench.__setitem__(f, m))(finger), 10)
            self.create_subscription(
                WrenchStamped, f'/rh8d/right/fingertip/{finger}/wrench_display',
                (lambda f: lambda m: self.display.__setitem__(f, m))(finger), 10)
        self.pub = self.create_publisher(AllSensors, 'R_AllSensors', 10)
        self.tare = self.create_client(Trigger, f'/{NODE_NAME}/tare')

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, readings):
        """readings: {sensor id: (fx, fy, fz)}"""
        msg = AllSensors()
        msg.header.stamp = self.get_clock().now().to_msg()
        for sensor_id, (fx, fy, fz) in readings.items():
            sensor = LoneSensor()
            sensor.id = sensor_id
            sensor.fx, sensor.fy, sensor.fz = fx, fy, fz
            sensor.is_present = True
            msg.data.append(sensor)
        msg.length = len(msg.data)
        self.pub.publish(msg)

    def send_all(self, reading, times=1):
        for _ in range(times):
            self.send({i: reading for i in range(5)})
            self.spin(0.05)

    def force(self, finger='index', display=False):
        source = self.display if display else self.wrench
        f = source[finger].wrench.force
        return (f.x, f.y, f.z)


def start_adapter():
    return subprocess.Popen(
        ['ros2', 'run', 'seed_hand_bringup', 'sensor_wrench_adapter', '--ros-args',
         '-r', f'__node:={NODE_NAME}',
         '-p', 'side:=right', '-p', 'axis_map:=[z,-y,x]',
         '-p', 'tare_samples:=10', '-p', 'min_force:=10.0'],
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
    check = Checks('sensor_wrench_adapter with a faked FTS stream')
    adapter = start_adapter()
    h = Harness()
    h.spin(3.0)
    try:
        check.section('auto tare')
        h.send_all(REST, times=10)
        h.spin(1.0)
        check(not h.wrench, f'nothing is published while taring ({len(h.wrench)})')
        h.send_all(REST)
        h.spin(0.8)
        check(len(h.wrench) == 5, f'one wrench per finger after the tare '
                                  f'({len(h.wrench)})')
        check(sum(abs(v) for v in h.force()) < 1e-9,
              f'the rest level tares to zero ({h.force()})')
        check(h.wrench['index'].header.frame_id == 'r_index_fingertip',
              f'stamped in the fingertip frame '
              f'({h.wrench["index"].header.frame_id})')

        check.section('axis map and deadband')
        # axis_map [z,-y,x]: out.x = raw z, out.y = -raw y, out.z = raw x
        h.send({1: (REST[0] + 11, REST[1] + 22, REST[2] - 33)})
        h.spin(0.8)
        x, y, z = h.force()
        check(abs(x + 33) < 1e-6 and abs(y + 22) < 1e-6 and abs(z - 11) < 1e-6,
              f'axis_map [z,-y,x]: raw (11, 22, -33) -> ({x:.0f}, {y:.0f}, {z:.0f})')
        h.send({1: (REST[0] + 2, REST[1] + 2, REST[2] - 2)})
        h.spin(0.8)
        check(sum(abs(v) for v in h.force()) == 0.0,
              f'a magnitude below min_force deadbands to zero ({h.force()})')

        check.section('silent sensors and the display stream')
        h.send({1: (REST[0] + 11, REST[1] + 22, REST[2] - 33)})
        h.spin(0.5)
        h.send({0: REST})               # sensor 1 omitted entirely
        h.spin(0.8)
        check(sum(abs(v) for v in h.force()) == 0.0,
              f'a sensor missing from the message publishes zeros, not its '
              f'last value ({h.force()})')
        h.send({1: (REST[0], REST[1], REST[2] + 60)})     # raw +z: not physical
        h.spin(0.8)
        check(abs(h.force()[0] - 60) < 1e-6,
              f'the raw topic keeps the positive-z reading ({h.force()[0]:.0f})')
        check(abs(h.force(display=True)[0]) < 1e-9,
              f'wrench_display clips it to zero '
              f'({h.force(display=True)[0]:.0f})')

        check.section('tare service and live parameters')
        available = h.tare.wait_for_service(timeout_sec=8.0)
        check(available, 'the tare service is advertised')
        if available:
            future = h.tare.call_async(Trigger.Request())
            end = time.time() + 8
            while time.time() < end and not future.done():
                rclpy.spin_once(h, timeout_sec=0.1)
            check(future.done() and future.result().success,
                  'the tare service reports success')
            h.send_all((500, 500, 500), times=10)
            h.spin(0.8)
            h.send({1: (500, 500, 500)})
            h.spin(0.8)
            check(sum(abs(v) for v in h.force()) == 0.0,
                  f're-taring rezeroes at the new rest level ({h.force()})')

        result = subprocess.run(
            ['ros2', 'param', 'set', f'/{NODE_NAME}', 'axis_map', "['q','y','z']"],
            capture_output=True, text=True, timeout=60)
        check('fail' in (result.stdout + result.stderr).lower()
              or result.returncode != 0,
              f'an invalid axis_map is rejected ({result.stdout.strip()})')
        h.send({1: (REST[0] + 11, REST[1] + 22, REST[2] - 33)})
        h.spin(0.8)
        check('index' in h.wrench,
              'and the node keeps running after the bad parameter')
    finally:
        stop(adapter)
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
