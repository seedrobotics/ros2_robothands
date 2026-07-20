from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration('side')
    coupling = LaunchConfiguration('finger_coupling')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('rh8d_description'), 'urdf', 'rh8d.urdf.xacro']),
            ' side:=', side,
            ' finger_coupling:=', coupling,
            ' couple_ring_little:=', LaunchConfiguration('couple_ring_little'),
        ]),
        value_type=str)

    sequential = IfCondition(PythonExpression(["'", coupling, "' == 'sequential'"]))
    not_sequential = UnlessCondition(PythonExpression(["'", coupling, "' == 'sequential'"]))

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('finger_coupling', default_value='mimic',
                              choices=['mimic', 'sequential', 'independent']),
        DeclareLaunchArgument('couple_ring_little', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # Plain setup: the GUI publishes /joint_states directly.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=not_sequential),

        # Sequential coupling: GUI output is re-routed through the clamper so
        # the staggered mimic offsets are clamped to the joint limits like the
        # physics engines do.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             remappings=[('joint_states', 'joint_states_raw')],
             condition=sequential),
        Node(package='rh8d_description', executable='mimic_joint_clamper.py',
             condition=sequential),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('rh8d_description'),
                                                    'rviz', 'rh8d.rviz'])]),
    ])
