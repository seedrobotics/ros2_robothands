#!/usr/bin/env python3
# This sample assumes you are using a Right Hand. If not, change the R_sensor_user_command Topic name to L_sensor_user_command
# User sample code that sends a command to calibrate the sensor and sets its frequency to a new one.

import time

import rclpy
from rclpy.node import Node

from sensor_pkg.msg import SensorUserCommand

DESIRED_FREQUENCY = 20          #20Hz, can be changed with values between 1 and 50

class UserTalker(Node):
	def __init__(self) -> None:
		super().__init__("user_talker")

		# Publisher to R_sensor_user_command to be able to send command to the sensors
		self._pub = self.create_publisher(SensorUserCommand, "R_sensor_user_command", 10)

		# Let 1s for the node to initialize before publishing
		time.sleep(1)

		# Build the command message and publish it once
		command = SensorUserCommand()
		command.calibrate = True
		command.set_frequency = True
		command.frequency = DESIRED_FREQUENCY
		self._pub.publish(command)


def main() -> None:
	rclpy.init()
	node = UserTalker()
	try:
		# Spin briefly so the publish can be processed before shutdown
		rclpy.spin_once(node, timeout_sec=0.1)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
