#!/usr/bin/env python3
# User sample: send a single position+speed command to all 8 RH8D joints.

import rclpy
from rclpy.node import Node
from seed_robotics.msg import JointListSetSpeedPos, JointSetSpeedPos


joint_names        = ['r_w_rotation', 'r_w_flexion', 'r_w_adduction',
                      'r_th_adduction', 'r_th_flexion',
                      'r_ix_flexion', 'r_middle_flexion', 'r_ring_ltl_flexion']
target_positions   = [1000, 2048, 2000, 0, 0, 0, 0, 0]
target_speeds      = [0, 100, 200, 300, 400, 500, 0, 0]


def main(args=None):
    rclpy.init(args=args)
    node = Node('joint_speed_position_setter')

    pub = node.create_publisher(JointListSetSpeedPos, 'R_speed_position', 10)

    # Build message
    joints_list = []
    for name, pos, spd in zip(joint_names, target_positions, target_speeds):
        j = JointSetSpeedPos()
        j.name        = name
        j.target_pos  = pos
        j.target_speed = spd
        joints_list.append(j)

    msg = JointListSetSpeedPos()
    msg.joints = joints_list

    # Allow ROS 2 to discover the subscriber before publishing
    import time
    time.sleep(1.0)
    pub.publish(msg)
    node.get_logger().info('Published speed/position command')

    # Spin briefly so the message is actually sent
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
