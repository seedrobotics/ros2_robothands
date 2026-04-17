from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sensors_arg = DeclareLaunchArgument(
        'use_sensors',
        default_value='True',
        description='Whether to launch the sensor node',
    )

    hand_config_arg = DeclareLaunchArgument(
        'hand_config',
        default_value='RH8D_L.yaml',
        description='Hand configuration file (RH8D_L.yaml for left, RH8D_R.yaml for right)',
    )

    hand_config = PathJoinSubstitution([
        FindPackageShare('seed_robotics'),
        'config',
        LaunchConfiguration('hand_config'),
    ])

    # Hand controller node
    hand_node = Node(
        package='seed_robotics',
        executable='hand_handle_node',
        name='hand_handle_node',
        output='screen',
        parameters=[hand_config],
    )

    # Sensor launch file (conditional)
    sensor_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('sensor_pkg'),
            'launch',
            'Sensors.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('use_sensors')),
    )

    return LaunchDescription([
        use_sensors_arg,
        hand_config_arg,
        hand_node,
        sensor_launch,
    ])
