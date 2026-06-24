from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    normalized_arg = DeclareLaunchArgument(
        'normalized_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Publish/accept normalized joint topics (radians for wrist, '
                    '0-1 closedness for fingers). Off by default.',
    )

    config = PathJoinSubstitution([
        FindPackageShare('seed_robotics'),
        'config',
        'RH8D_R.yaml',
    ])

    hand_node = Node(
        package='seed_robotics',
        executable='hand_handle_node',   # was hand_handle_node.py
        name='hand_handle_node',
        output='screen',
        parameters=[
            config,
            {'normalized_mode': ParameterValue(
                LaunchConfiguration('normalized_mode'), value_type=bool)},
        ],
    )

    return LaunchDescription([normalized_arg, hand_node])
