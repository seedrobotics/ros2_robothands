#!/usr/bin/env python3
"""Bridge a joint_state_publisher GUI to a JointTrajectoryController.

Lets the slider panel command the simulated hand in any coupling mode:

    jsp_gui --gui_joint_states--> this node --joint_trajectory--> controller

The controller's joint set is learned from its controller_state topic, so
slider messages are filtered and ordered automatically (jsp_gui also
publishes mimic-follower values, which the controller must not receive).
Commands are clamped a small margin inside the joint limits: dartsim pins
joints that park exactly ON a limit (stuck until knocked free).
"""
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GuiToTrajectory(Node):

    def __init__(self):
        super().__init__('joint_gui_to_trajectory')
        self.tfs = self.declare_parameter('time_from_start', 0.2).value
        self.margin = self.declare_parameter('limit_margin', 0.03).value
        self.joints = None
        self.limits = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_urdf, qos)
        self.create_subscription(JointTrajectoryControllerState,
                                 'hand_controller/controller_state', self.on_state, 1)
        self.create_subscription(JointState, 'gui_joint_states', self.on_gui, 10)
        self.pub = self.create_publisher(JointTrajectory, 'hand_controller/joint_trajectory', 10)

    def on_urdf(self, msg):
        for joint in ET.fromstring(msg.data).findall('joint'):
            limit = joint.find('limit')
            if limit is not None and limit.get('lower') is not None:
                self.limits[joint.get('name')] = (
                    float(limit.get('lower')) + self.margin,
                    float(limit.get('upper')) - self.margin)

    def on_state(self, msg):
        if self.joints != list(msg.joint_names):
            self.joints = list(msg.joint_names)
            self.get_logger().info(f'commanding {len(self.joints)} joints: {self.joints}')

    def on_gui(self, msg):
        if not self.joints:
            return
        values = dict(zip(msg.name, msg.position))
        if not all(j in values for j in self.joints):
            return
        traj = JointTrajectory()
        traj.joint_names = self.joints
        def clamped(j):
            lo, hi = self.limits.get(j, (-1e9, 1e9))
            return min(max(values[j], lo), hi)
        traj.points = [JointTrajectoryPoint(
            positions=[clamped(j) for j in self.joints],
            time_from_start=Duration(sec=int(self.tfs), nanosec=int((self.tfs % 1) * 1e9)))]
        self.pub.publish(traj)


def main():
    rclpy.init()
    try:
        rclpy.spin(GuiToTrajectory())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
