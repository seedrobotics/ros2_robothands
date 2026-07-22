#!/usr/bin/env python3
"""Convert the simulated palm IR lidar scan into a Range reading that behaves
like the real RH8D palm sensor: a single distance value, and 255 mm (0.255 m)
when nothing is in range - the real sensor never reports infinity.

    palm IR gz lidar --/scan (LaserScan)--> palm_ir_adapter --/range (Range)-->
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class PalmIrAdapter(Node):

    def __init__(self):
        super().__init__('palm_ir_adapter')
        self.no_echo = self.declare_parameter('no_echo_range', 0.255).value
        self.pub = self.create_publisher(Range, 'range', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg):
        finite = [r for r in msg.ranges
                  if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        out = Range()
        out.header = msg.header
        out.radiation_type = Range.INFRARED
        out.field_of_view = 0.05
        out.min_range = msg.range_min
        out.max_range = msg.range_max
        out.range = sum(finite) / len(finite) if finite else self.no_echo
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(PalmIrAdapter())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
