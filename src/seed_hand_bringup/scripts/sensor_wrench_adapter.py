#!/usr/bin/env python3
"""FTS fingertip sensors -> per-finger geometry_msgs/WrenchStamped.

Converts the sensor driver's AllSensors array into one WrenchStamped per
fingertip on the same topics the Gazebo simulation publishes:

    /rh8d/<side>/fingertip/<finger>/wrench

stamped in the model's fingertip frames (<p><finger>_fingertip), so RViz can
display live force arrows on the hand and any standard tool (PlotJuggler,
rosbag, ...) can consume the data without knowing the Seed message types.

Sensors are matched by their id field (wire index), not array position - the
driver omits absent sensors from the array. finger_order maps wire index ->
finger name; adjust it if the sensors are cabled differently.

Values are published in the sensor's own (factory-calibrated) units.
Arrow length in RViz is a display concern - tune Force Arrow Scale in
rh8d.rviz, not here.

Processing pipeline (sensor values -> published wrench):
  1. tare      subtract the per-sensor rest bias. Auto-tared from the first
               tare_samples messages after startup (keep the fingertips
               unloaded!); re-zero anytime:
                   ros2 service call /sensor_wrench_adapter_right/tare \
                       std_srvs/srv/Trigger
  2. remap     axis_map rotates the sensor axes into the fingertip frame,
               e.g. ['y', '-x', 'z'] means: fingertip x = sensor y,
               fingertip y = -sensor x, fingertip z = sensor z.
  3. deadband  |F| < min_force (sensor units) publishes a zero wrench,
               so noise does not draw arrows in RViz. 0 disables.
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

from ros2_sensor_pkg.msg import AllSensors

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']
AXES = {'x': 0, 'y': 1, 'z': 2}


def parse_axis_map(axis_map):
    """['y', '-x', 'z'] -> [(src_index, sign), ...] for output axes x,y,z."""
    if len(axis_map) != 3:
        raise ValueError('axis_map needs exactly 3 entries')
    remap = []
    for spec in axis_map:
        sign = -1.0 if spec.startswith('-') else 1.0
        remap.append((AXES[spec.lstrip('-')], sign))
    return remap


class SensorWrenchAdapter(Node):

    def __init__(self):
        super().__init__('sensor_wrench_adapter')
        side = self.declare_parameter('side', 'right').value
        order = self.declare_parameter('finger_order', FINGERS).value
        self.min_force = self.declare_parameter('min_force', 0.0).value
        self.tare_samples = self.declare_parameter('tare_samples', 25).value
        axis_map = self.declare_parameter('axis_map', ['x', 'y', 'z']).value
        topic_prefix = {'left': 'L_', 'right': 'R_'}[side]
        jp = side[0] + '_'

        self.remap = parse_axis_map(axis_map)
        # min_force and axis_map are live-tunable:
        #   ros2 param set /sensor_wrench_adapter_right axis_map "['y','-x','z']"
        self.add_on_set_parameters_callback(self.on_set_params)

        self.frame_by_id = {i: f'{jp}{f}_fingertip' for i, f in enumerate(order)}
        self.pub_by_id = {
            i: self.create_publisher(
                WrenchStamped, f'/rh8d/{side}/fingertip/{f}/wrench', 10)
            for i, f in enumerate(order)}

        self.bias = {}        # id -> (fx, fy, fz) rest offset in counts
        self._tare_acc = {}   # id -> [n, sum_fx, sum_fy, sum_fz]
        self._taring = self.tare_samples > 0

        self.create_subscription(AllSensors, topic_prefix + 'AllSensors',
                                 self.on_sensors, 10)
        self.create_service(Trigger, '~/tare', self.on_tare)
        self.get_logger().info(
            f'publishing /rh8d/{side}/fingertip/<finger>/wrench for {order} '
            f'(min_force={self.min_force}, axis_map={axis_map}, '
            f'auto-taring over first {self.tare_samples} samples - keep '
            'fingertips unloaded)')

    def on_set_params(self, params):
        for p in params:
            if p.name == 'min_force':
                self.min_force = float(p.value)
            elif p.name == 'axis_map':
                try:
                    self.remap = parse_axis_map(p.value)
                except (ValueError, KeyError) as e:
                    return SetParametersResult(
                        successful=False,
                        reason=f'bad axis_map (want e.g. ["y","-x","z"]): {e}')
            self.get_logger().info(f'{p.name} -> {p.value}')
        return SetParametersResult(successful=True)

    def on_tare(self, request, response):
        self._tare_acc.clear()
        self._taring = True
        response.success = True
        response.message = f'taring over the next {self.tare_samples} samples'
        return response

    def _accumulate_tare(self, sensors):
        done = True
        for s in sensors:
            acc = self._tare_acc.setdefault(s.id, [0, 0.0, 0.0, 0.0])
            if acc[0] < self.tare_samples:
                acc[0] += 1
                acc[1] += s.fx
                acc[2] += s.fy
                acc[3] += s.fz
            if acc[0] < self.tare_samples:
                done = False
        if done and self._tare_acc:
            self.bias = {sid: (a[1] / a[0], a[2] / a[0], a[3] / a[0])
                         for sid, a in self._tare_acc.items()}
            self._taring = False
            self.get_logger().info(
                'tare done: ' + ', '.join(
                    f'{sid}:({b[0]:.0f},{b[1]:.0f},{b[2]:.0f})'
                    for sid, b in sorted(self.bias.items())))

    def on_sensors(self, msg):
        present = [s for s in msg.data
                   if s.is_present and s.id in self.pub_by_id]
        if self._taring:
            self._accumulate_tare(present)
            return  # publish nothing until the zero level is known
        forces = {}
        for s in present:
            bias = self.bias.get(s.id, (0.0, 0.0, 0.0))
            raw = (s.fx - bias[0], s.fy - bias[1], s.fz - bias[2])
            f = [sign * raw[src] for src, sign in self.remap]
            if math.hypot(*f) < self.min_force:
                f = [0.0, 0.0, 0.0]
            forces[s.id] = f
        # sensors below the firmware's transmit threshold are omitted from
        # the message entirely - publish zeros so arrows drop instead of
        # freezing at the last value
        for sid, pub in self.pub_by_id.items():
            w = WrenchStamped()
            w.header.stamp = msg.header.stamp
            w.header.frame_id = self.frame_by_id[sid]
            f = forces.get(sid, (0.0, 0.0, 0.0))
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = f
            pub.publish(w)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(SensorWrenchAdapter())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
