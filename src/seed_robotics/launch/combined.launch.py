from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    hand = LaunchConfiguration('hand').perform(context)   # 'left' or 'right'
    suffix = 'L' if hand == 'left' else 'R'               # -> RH8D_L.yaml / RH8D_R.yaml

    # Hand controller config: config/RH8D_<L|R>.yaml
    hand_config = PathJoinSubstitution([
        FindPackageShare('seed_robotics'),
        'config',
        f'RH8D_{suffix}.yaml',
    ])

    hand_node = Node(
        package='seed_robotics',
        executable='hand_handle_node',
        name='hand_handle_node',
        output='screen',
        parameters=[
            hand_config,
            {'normalized_mode': ParameterValue(
                LaunchConfiguration('normalized_mode'), value_type=bool)},
        ],
    )

    # Sensor launch (conditional). Sensors.launch.py selects
    # config/sensors_<left|right>.yaml from the same 'hand' value.
    sensor_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros2_sensor_pkg'),
            'launch',
            'Sensors.launch.py',
        ]),
        launch_arguments={'hand': hand}.items(),
        condition=IfCondition(LaunchConfiguration('use_sensors')),
    )

    return [hand_node, sensor_launch]


def generate_launch_description():
    hand_arg = DeclareLaunchArgument(
        'hand',
        default_value='left',
        choices=['left', 'right'],
        description='Which hand to launch. Selects config/RH8D_L.yaml or '
                    'RH8D_R.yaml for the controller and config/sensors_left.yaml '
                    'or sensors_right.yaml for the sensor node.',
    )

    use_sensors_arg = DeclareLaunchArgument(
        'use_sensors',
        default_value='True',
        description='Whether to launch the sensor node',
    )

    normalized_arg = DeclareLaunchArgument(
        'normalized_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Publish/accept normalized joint topics (radians for wrist, '
                    '0-1 closedness for fingers). Off by default.',
    )

    return LaunchDescription([
        hand_arg,
        use_sensors_arg,
        normalized_arg,
        OpaqueFunction(function=launch_setup),
    ])
