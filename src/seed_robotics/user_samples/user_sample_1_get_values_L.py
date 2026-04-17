#!/usr/bin/env python3
# User sample: subscribe to joint states and print them continuously.
# For a RH8D Right Hand (prefix R_).

import rclpy
from rclpy.node import Node
from seed_robotics.msg import AllJoints


class JointsListener(Node):

    def __init__(self):
        super().__init__('joints_listener')
        self.create_subscription(AllJoints, 'L_Joints', self._cb, 10)

    def _cb(self, msg: AllJoints):
        print(msg.header)
        print('Number of joints: %d' % msg.length)
        for joint in msg.joints:
            print('Joint name        : %s' % joint.name)
            print('Joint ID          : %d' % joint.bus_id)
            print('Stiffness         : %d' % joint.stiffness)
            print('Stress Level      : %d' % joint.stress_level)
            print('Target Position   : %d' % joint.target_position)
            print('Target Speed      : %d' % joint.target_speed)
            print('Torque Limit      : %d' % joint.torque_limit)
            print('Present Position  : %d' % joint.present_position)
            print('Present Speed     : %d' % joint.present_speed)
            print('Temperature       : %d' % joint.temperature)
            print('Moving            : %d' % joint.moving)
            print('HW Error Condition: %d' % joint.hw_error_condition)
            print('Present Current   : %d' % joint.current)
            print('---')


def main(args=None):
    rclpy.init(args=args)
    node = JointsListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
