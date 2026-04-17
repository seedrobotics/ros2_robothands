from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='seed_fts3_right',
        description='Name of the sensor node',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('sensor_pkg'), 'config', 'sensors_right.yaml']
        ),
        description='Path to the YAML parameter file',
    )

    sensor_node = Node(
        package='sensor_pkg',
        executable='read_publish_sensor_node.py',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        node_name_arg,
        config_file_arg,
        sensor_node,
    ])
