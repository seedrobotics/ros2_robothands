#!/usr/bin/env python3

# This sample is done for a RH8D Right Hand
# You can adapt it to a right Hand by changing the 'l' by a 'r' on joint names
# User sample code High Level Logic : When an object is close to the Hand, the fingers close to grab the object. Once it's grabbed, the finger open themselves 5seconds later
# Several things are done here :
# - Get real-time data for each joint
# - Get real-time data from the IR sensor in the palm of the hand
# - Send the instruction to close the index, the ring and the little fingers
# - Send the instruction to close the thumb
# - Continuously checking if any joint is too stressed, i.e. if its current goes above the current limit hardcoded to 300mA
# - If a joint is too stressed, send an instruction to set its target position to its present position
# - When every joint have their present position equal to their target position -> open the finger and let the object go

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from seed_robotics.msg import (
    AllJoints,
    AllMainBoards,
    JointListSetSpeedPos,
    JointSetSpeedPos,
    LoneJoint,
)

# Hardcoded value of the current limit, if a joint goes above that limit it must stop forcing
CURRENT_LIMIT = 300 #mAmp

# Control class to be able to change the IR sensor value in the callback function
# Including 2 flags to control the main loop
class Control:
    def __init__(self):
        self.IR_sensor_value = 254
        self.start_flag = False
        self.step2_flag = False


class GrabObjectNode(Node):
    def __init__(self):
        super().__init__('listener')

        self.control = Control()
        # We will only need information about 5 joints.
        self.joint_list = [LoneJoint() for _ in range(5)]

        self.names_step_1 = [
            'r_th_adduction',
            'r_ix_flexion',
            'r_middle_flexion',
            'r_ring_ltl_flexion',
        ]
        self.target_positions_step_1 = [4095, 4095, 4095, 4095]
        self.target_speeds_step_1 = [0, 100, 100, 100]

        self.names_step_2 = ['r_th_flexion']
        self.target_positions_step_2 = [4095]
        self.target_speeds_step_2 = [50]

        self.names_step_3 = [
            'r_th_adduction',
            'r_th_flexion',
            'r_ix_flexion',
            'r_middle_flexion',
            'r_ring_ltl_flexion',
        ]
        self.target_positions_step_3 = [0, 0, 0, 0, 0]
        self.target_speeds_step_3 = [10, 10, 10, 10, 10]

        self.list_stressed_joints = []
        self.step1_sent_time = None
        self.release_time = None

        self.create_subscription(AllJoints, 'R_Joints', self.joints_callback, 10)
        self.create_subscription(AllMainBoards, 'R_Main_Boards', self.main_board_callback, 10)
        self.pub = self.create_publisher(JointListSetSpeedPos, 'R_speed_position', 10)

        self.timer = self.create_timer(0.1, self.loop)

    def build_speed_pos_msg(self, names, target_positions, target_speeds):
        joint_list = [JointSetSpeedPos() for _ in range(len(names))]
        for name, position, speed, joint in zip(names, target_positions, target_speeds, joint_list):
            joint.name = name
            joint.target_pos = position
            joint.target_speed = speed
        message = JointListSetSpeedPos()
        message.joints = joint_list
        return message

    def joints_callback(self, joints_data):
        for joint in joints_data.joints:
            if joint.name == 'r_th_adduction':
                self.joint_list[0] = joint
            if joint.name == 'r_th_flexion':
                self.joint_list[1] = joint
            if joint.name == 'r_ix_flexion':
                self.joint_list[2] = joint
            if joint.name == 'r_middle_flexion':
                self.joint_list[3] = joint
            if joint.name == 'r_ring_ltl_flexion':
                self.joint_list[4] = joint

    def main_board_callback(self, main_board_data):
        for board in main_board_data.boards:
            if board.name == 'r_main_board':
                self.control.IR_sensor_value = board.palm_ir_sensor

    def stop_stressing(self, joint):
        target_pos = joint.present_position
        joints = [JointSetSpeedPos()]
        joints[0].name = joint.name
        joints[0].target_pos = target_pos
        joints[0].target_speed = -1
        message = JointListSetSpeedPos()
        message.joints = joints
        self.pub.publish(message)

    def loop(self):
        now = self.get_clock().now()

        if not self.control.start_flag:
            if self.control.IR_sensor_value < 20:
                message = self.build_speed_pos_msg(
                    self.names_step_1,
                    self.target_positions_step_1,
                    self.target_speeds_step_1,
                )
                self.get_logger().info(str(message))
                self.pub.publish(message)
                self.control.start_flag = True
                self.step1_sent_time = now
            return

        if not self.control.step2_flag:
            if self.step1_sent_time is None or (now - self.step1_sent_time) < Duration(seconds=1.0):
                return
            message = self.build_speed_pos_msg(
                self.names_step_2,
                self.target_positions_step_2,
                self.target_speeds_step_2,
            )
            self.get_logger().info(str(message))
            self.pub.publish(message)
            self.control.step2_flag = True

        for joint in self.joint_list:
            if joint.current > CURRENT_LIMIT:
                if joint.name not in self.list_stressed_joints:
                    self.stop_stressing(joint)
                    self.list_stressed_joints.append(joint.name)
                    self.get_logger().info(str(self.list_stressed_joints))

        if len(self.list_stressed_joints) >= 3:
            if self.release_time is None:
                self.release_time = now + Duration(seconds=5.0)
                return
            if now >= self.release_time:
                message = self.build_speed_pos_msg(
                    self.names_step_3,
                    self.target_positions_step_3,
                    self.target_speeds_step_3,
                )
                self.get_logger().info(str(message))
                self.pub.publish(message)
                rclpy.shutdown()


def main():
    rclpy.init()
    node = GrabObjectNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
