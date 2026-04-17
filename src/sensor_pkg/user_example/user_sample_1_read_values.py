#!/usr/bin/env python3

# PLEASE NOTE : This samples assume you are using a RIGHT Hand
# To use a left hand you must replace the Topic prefix, i.e replace R_sensor_user_command by L_sensor_user_command
# Also change R_AllSensors to L_AllSensors
# User sample code that calibrates the sensor via a ROS message, recovers X Y Z sensor values and display them

import time

import rclpy
from rclpy.node import Node

from sensor_pkg.msg import AllSensors, LoneSensor, SensorUserCommand

class UserListener(Node):
    def __init__(self) -> None:
        super().__init__("user_listener_1")

        # Publisher to send a SensorUserCommand to the sensors
        self._pub = self.create_publisher(SensorUserCommand, "R_sensor_user_command", 10)

        # Subscribe to the AllSensors Topic, where sensor readings are published
        self._sub = self.create_subscription(AllSensors, "R_AllSensors", self._callback, 10)

        # Send calibration command after a short delay to let ROS2 initialize
        time.sleep(1)
        command = SensorUserCommand()
        command.calibrate = True
        self._pub.publish(command)

    # Callback function definition
    def _callback(self, sensor_data: AllSensors) -> None:
        # Initialize a list of LoneSensor messages to store the data that will be read
        sensor_values = [LoneSensor() for _ in range(sensor_data.length)]

        for i, sensor in enumerate(sensor_data.data):
            if sensor.is_present is False:  # Check if the sensor is present
                # if not: set id to None, sensor will be displayed as "Sensor None"
                sensor_values[i].id = None
            else:
                # If sensor is present, then copy the informations in the lone_sensor message
                sensor_values[i] = sensor

            # Then print each sensor with its X Y Z coordinates
            print("\t Sensor {} \n".format(sensor_values[i].id))
            print("X \t Y \t Z \n")
            print("{} \t {} \t {} \n".format(sensor_values[i].fx, sensor_values[i].fy, sensor_values[i].fz))


def main() -> None:
    rclpy.init()
    node = UserListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
