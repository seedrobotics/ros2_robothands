#!/usr/bin/env python3
"""Joint-state republisher that enforces mimic relations WITH limit clamping.

joint_state_publisher computes mimic followers as multiplier * leader + offset
but does not clamp the result to the follower's joint limits. For the RH8D's
'sequential' finger coupling the staggered negative offsets rely on exactly
that clamping (which physics engines do apply), so RViz visualization needs
this node in between:

    joint_state_publisher_gui  --/joint_states_raw-->  mimic_joint_clamper
        --/joint_states-->  robot_state_publisher

The robot description is taken from the 'robot_description' parameter or, if
unset, from the /robot_description topic published by robot_state_publisher.
"""
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class MimicJointClamper(Node):

    def __init__(self):
        super().__init__('mimic_joint_clamper')
        self.mimics = {}
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(JointState, 'joint_states_raw', self.on_joint_states, 10)

        urdf = self.declare_parameter('robot_description', '').value
        if urdf:
            self.parse(urdf)
        else:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.desc_sub = self.create_subscription(String, 'robot_description', self.on_description, qos)

    def on_description(self, msg):
        self.parse(msg.data)

    def parse(self, urdf):
        self.mimics = {}
        for joint in ET.fromstring(urdf).findall('joint'):
            mimic = joint.find('mimic')
            if mimic is None:
                continue
            limit = joint.find('limit')
            self.mimics[joint.get('name')] = {
                'leader': mimic.get('joint'),
                'multiplier': float(mimic.get('multiplier', 1.0)),
                'offset': float(mimic.get('offset', 0.0)),
                'lower': float(limit.get('lower', -math.inf)) if limit is not None else -math.inf,
                'upper': float(limit.get('upper', math.inf)) if limit is not None else math.inf,
            }
        self.get_logger().info(f'tracking {len(self.mimics)} mimic joints')

    def on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        out = JointState()
        out.header = msg.header
        for name, pos in positions.items():
            if name not in self.mimics:
                out.name.append(name)
                out.position.append(pos)
        for name, m in self.mimics.items():
            if m['leader'] in positions:
                value = m['multiplier'] * positions[m['leader']] + m['offset']
                out.name.append(name)
                out.position.append(min(max(value, m['lower']), m['upper']))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MimicJointClamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
