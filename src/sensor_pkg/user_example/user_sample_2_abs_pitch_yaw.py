#!/usr/bin/env python3

# User sample code that calibrates the sensor via a ROS message, recovers the polar coordinates sensor values and display them

import math
import time

import rclpy
from rclpy.node import Node

from sensor_pkg.msg import AllSensors, LoneSensor, SensorUserCommand

#Flag to display value in degrees or radiant
DEGREES = True  # If true, display in degrees

class SensorListener(Node):
    def __init__(self) -> None:
        super().__init__("sensor_listener")

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
        # Initialize a list of LoneSensor messages to store the data that are read
        sensor_values = [LoneSensor() for _ in range(sensor_data.length)]
        for i, sensor in enumerate(sensor_data.data):
            if sensor.is_present is False:  # Check if the sensor is present
                # if not: set id to None, sensor will be displayed as "Sensor None"
                sensor_values[i].id = None
            else:
                # If sensor is present, then copy the informations in the LoneSensor message
                sensor_values[i] = sensor

            # Then print each sensor with its polar coordinates
            print("\t Sensor {} \n".format(sensor_values[i].id))
            if DEGREES:
                print("mN_Value \t \t deg_Pitch \t \t \t deg_Yaw \n")
                print("{} \t {} \t {} \n".format(
                    sensor_values[i].abs,
                    math.degrees(sensor_values[i].pitch),
                    math.degrees(sensor_values[i].yaw),
                ))
            else:
                print("mN_Value \t \t rad_Pitch \t \t \t rad_Yaw \n")
                print("{} \t {} \t {} \n".format(
                    sensor_values[i].abs,
                    sensor_values[i].pitch,
                    sensor_values[i].yaw,
                ))


def main() -> None:
    rclpy.init()
    node = SensorListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
