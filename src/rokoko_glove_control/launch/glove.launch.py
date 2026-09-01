"""Drive an RH8D from a Rokoko Smartglove.

    ros2 launch rokoko_glove_control glove.launch.py side:=right

Publishes `motor_commands` in aligned units, so it drives whichever hand is
already up - run it alongside either of:

    ros2 launch seed_hand_bringup hand.launch.py side:=right     # real hand
    ros2 launch seed_rh8d_gazebo gazebo.launch.py side:=right \
        finger_coupling:=independent                             # simulation

The Rokoko hand solver has to be running first; it is what turns raw sensor
poses into the skeleton this reads:

    ~/.local/share/rokoko-device-sdk/bin/rkk-hand-solver

Wrist rotation follows your FOREARM, not your wrist - pronation happens in
the forearm, which is why the wrist joint's own twist barely moves. It is
tared when the node starts, so hold your forearm in a neutral pose as it
comes up; restart the node to re-zero it.

CAUTION on hardware: the hand follows your hand as soon as the glove is seen.
`max_range_per_second` bounds how fast it gets there (an axis's full range
takes 1/3 s by default), but it still moves to wherever your hand is - so
start with your hand somewhere sane.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    side = LaunchConfiguration('side')

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right',
                              choices=['left', 'right', 'auto'],
                              description='Which glove hand to follow'),
        DeclareLaunchArgument('joint_prefix', default_value='',
                              description='Robot joint prefix; defaults to '
                                          'matching side (r_ / l_)'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Glove calibration JSON; empty picks '
                                          'up rh8d_calibration.json beside the '
                                          'glove modules, where --calibrate '
                                          'writes it'),
        DeclareLaunchArgument('solver_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rate', default_value='50.0'),
        DeclareLaunchArgument('lock_wrist', default_value='false',
                              description='Publish a centred wrist and follow '
                                          'the fingers only'),
        DeclareLaunchArgument('wrist_scale', default_value='1.0'),
        DeclareLaunchArgument('max_range_per_second', default_value='3.0',
                              description='Slew limit, axis ranges per second; '
                                          '0 disables it'),

        Node(package='rokoko_glove_control', executable='glove_node',
             name='rokoko_glove_node',
             output='screen',
             parameters=[{
                 'side': side,
                 'joint_prefix': PythonExpression(
                     ["'", LaunchConfiguration('joint_prefix'), "' or "
                      "('l_' if '", side, "' == 'left' else 'r_')"]),
                 'calibration_file': LaunchConfiguration('calibration_file'),
                 'solver_host': LaunchConfiguration('solver_host'),
                 'rate': LaunchConfiguration('rate'),
                 'lock_wrist': LaunchConfiguration('lock_wrist'),
                 'wrist_scale': LaunchConfiguration('wrist_scale'),
                 'max_range_per_second':
                     LaunchConfiguration('max_range_per_second'),
             }]),
    ])
