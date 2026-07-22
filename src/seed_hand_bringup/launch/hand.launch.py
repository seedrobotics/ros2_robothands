"""Bring up a Seed Robotics hand: driver node plus optional FTS sensors.

    ros2 launch seed_hand_bringup hand.launch.py side:=right use_sensors:=true

side:=both drives two hands on one serial port (see RH8D_RL.yaml) and, with
use_sensors:=true, starts one sensor node per hand. Serial ports must be set
in the driver and sensor YAML configs first.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HAND_CONFIG = {'left': 'RH8D_L.yaml', 'right': 'RH8D_R.yaml', 'both': 'RH8D_RL.yaml'}


def setup(context, *args, **kwargs):
    side = context.launch_configurations['side']
    use_sensors = context.launch_configurations['use_sensors'].lower() == 'true'

    actions = [
        Node(
            package='seed_hand_driver',
            executable='hand_handle_node',
            name='hand_handle_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('seed_hand_driver'), 'config', HAND_CONFIG[side]])],
        ),
    ]

    if use_sensors:
        sensor_sides = ['left', 'right'] if side == 'both' else [side]
        for s in sensor_sides:
            actions.append(IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('ros2_sensor_pkg'), 'launch', 'Sensors.launch.py']),
                launch_arguments={
                    'node_name': f'seed_fts3_{s}',
                    'config_file': PathJoinSubstitution([
                        FindPackageShare('ros2_sensor_pkg'), 'config', f'sensors_{s}.yaml']),
                }.items(),
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right',
                              choices=['left', 'right', 'both'],
                              description='Which hand(s) to bring up'),
        DeclareLaunchArgument('use_sensors', default_value='false',
                              description='Also start the in-hand FTS sensor node(s)'),
        OpaqueFunction(function=setup),
    ])
