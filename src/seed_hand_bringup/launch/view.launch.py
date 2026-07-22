"""Live RViz view of the real hand.

Run next to hand.launch.py (which provides motor_states via the aligned
interface). The measured motor positions are mapped through the tendon
coupling model to full phalanx joint states, so RViz shows the hand
closing/wrapping as the real one moves.

    ros2 launch seed_hand_bringup view.launch.py side:=right
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration('side')
    desc_pkg = FindPackageShare('seed_rh8d_description')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([desc_pkg, 'urdf', 'rh8d.urdf.xacro']),
            ' side:=', side,
            ' use_ros2_control:=false',
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right',
                              choices=['left', 'right']),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # measured motor axes -> full joint state through the tendon model
        Node(package='seed_rh8d_description', executable='rh8d_coupling_node.py',
             name='motor_state_mapper',
             remappings=[('motor_commands', 'motor_states')],
             parameters=[{'prefix': PythonExpression(
                              ["'l_' if '", side, "' == 'left' else 'r_'"]),
                          'output': 'joint_states'}]),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([desc_pkg, 'rviz', 'rh8d.rviz'])]),
    ])
