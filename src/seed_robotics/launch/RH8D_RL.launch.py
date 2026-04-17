from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('seed_robotics'),
        'config',
        'RH8D_RL.yaml',
    ])

    hand_node = Node(
        package='seed_robotics',
        executable='hand_handle_node',
        name='hand_handle_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([hand_node])
