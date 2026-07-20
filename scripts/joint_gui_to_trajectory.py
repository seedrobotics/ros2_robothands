#!/usr/bin/env python3
"""Bridge a joint_state_publisher GUI to a JointTrajectoryController.

Lets the slider panel command the simulated hand in any coupling mode:

    jsp_gui --gui_joint_states--> this node --joint_trajectory--> controller

The controller's joint set is learned from its controller_state topic, so
slider messages are filtered and ordered automatically (jsp_gui also
publishes mimic-follower values, which the controller must not receive).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GuiToTrajectory(Node):

    def __init__(self):
        super().__init__('joint_gui_to_trajectory')
        self.tfs = self.declare_parameter('time_from_start', 0.2).value
        self.joints = None
        self.create_subscription(JointTrajectoryControllerState,
                                 'hand_controller/controller_state', self.on_state, 1)
        self.create_subscription(JointState, 'gui_joint_states', self.on_gui, 10)
        self.pub = self.create_publisher(JointTrajectory, 'hand_controller/joint_trajectory', 10)

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
        traj.points = [JointTrajectoryPoint(
            positions=[values[j] for j in self.joints],
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
