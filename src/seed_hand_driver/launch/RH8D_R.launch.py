from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('seed_hand_driver'),
        'config',
        'RH8D_R.yaml',
    ])

    hand_node = Node(
        package='seed_hand_driver',
        executable='hand_handle_node',   # was hand_handle_node.py
        name='hand_handle_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([hand_node])
