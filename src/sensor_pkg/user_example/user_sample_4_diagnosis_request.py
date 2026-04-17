#!/usr/bin/env python3

# This sample assumes you are using a Right Hand. If not, change the R_sensor_user_command Topic name to L_sensor_user_command
# User sample code that sends a command to request sensor diagnosis and prints the answer

import time

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log
from sensor_pkg.msg import SensorUserCommand

DESIRED_FREQUENCY = 20          #1Hz, can be changed with values between 1 and 50

class UserTalker(Node):
    def __init__(self) -> None:
        super().__init__("user_talker")

        # Publisher to sensor_user_command to be able to send command to the sensors
        self._pub = self.create_publisher(SensorUserCommand, "R_sensor_user_command", 10)

        # Subscribe to the rosout topic to read Log messages
        self._sub = self.create_subscription(Log, "rosout", self._callback, 10)

        # Let 1s to ROS for the Node's initialization before publishing
        time.sleep(1)

        # Build the command message and publish it once
        command = SensorUserCommand()
        command.diagnosis_request = True
        self._pub.publish(command)

    # Defining the callback function for the rosout topic
    def _callback(self, log_data: Log) -> None:
        # Print only the 'msg' field instead of the whole Log message
        print(log_data.msg)


def main() -> None:
    rclpy.init()
    node = UserTalker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
