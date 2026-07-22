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

force_scale converts raw counts to the published unit. The default 0.01 is
a ROUGH pre-calibration that brings typical count magnitudes near the
newton range the sim publishes (so shared RViz/PlotJuggler configs work);
replace it with the measured counts-per-newton factor once calibrated.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

from ros2_sensor_pkg.msg import AllSensors

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']


class SensorWrenchAdapter(Node):

    def __init__(self):
        super().__init__('sensor_wrench_adapter')
        side = self.declare_parameter('side', 'right').value
        order = self.declare_parameter('finger_order', FINGERS).value
        self.scale = self.declare_parameter('force_scale', 0.01).value
        topic_prefix = {'left': 'L_', 'right': 'R_'}[side]
        jp = side[0] + '_'

        self.finger_by_id = dict(enumerate(order))
        self.frame_by_id = {i: f'{jp}{f}_fingertip' for i, f in enumerate(order)}
        self.pub_by_id = {
            i: self.create_publisher(
                WrenchStamped, f'/rh8d/{side}/fingertip/{f}/wrench', 10)
            for i, f in enumerate(order)}

        self.create_subscription(AllSensors, topic_prefix + 'AllSensors',
                                 self.on_sensors, 10)
        self.get_logger().info(
            f'publishing /rh8d/{side}/fingertip/<finger>/wrench for '
            f'{order} (force_scale={self.scale}, rough default - not a '
            'measured counts-per-newton calibration)')

    def on_sensors(self, msg):
        for s in msg.data:
            if not s.is_present or s.id not in self.pub_by_id:
                continue
            w = WrenchStamped()
            w.header.stamp = msg.header.stamp
            w.header.frame_id = self.frame_by_id[s.id]
            w.wrench.force.x = s.fx * self.scale
            w.wrench.force.y = s.fy * self.scale
            w.wrench.force.z = s.fz * self.scale
            self.pub_by_id[s.id].publish(w)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(SensorWrenchAdapter())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
