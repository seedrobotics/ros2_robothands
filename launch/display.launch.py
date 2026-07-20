from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration('side')
    coupling = LaunchConfiguration('finger_coupling')
    use_coupling = LaunchConfiguration('use_coupling')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('rh8d_description'), 'urdf', 'rh8d.urdf.xacro']),
            ' side:=', side,
            ' finger_coupling:=', coupling,
            ' couple_ring_little:=', LaunchConfiguration('couple_ring_little'),
        ]),
        value_type=str)

    coupling_demo = PythonExpression(
        ["'", coupling, "' == 'independent' and '", use_coupling, "' == 'true'"])
    plain = IfCondition(PythonExpression(
        ["'", coupling, "' == 'mimic' or ('", coupling, "' == 'independent'",
         " and '", use_coupling, "' != 'true')"]))

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('finger_coupling', default_value='mimic',
                              choices=['mimic', 'independent']),
        DeclareLaunchArgument('couple_ring_little', default_value='true'),
        DeclareLaunchArgument('use_coupling', default_value='true',
                              description='In independent mode: drive the hand '
                                          'through the tendon coupling node with '
                                          'one slider per motor'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # Plain setup: the GUI publishes /joint_states directly.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=plain),

        # Independent + coupling demo: 8 motor sliders (from the generated
        # motor panel description) drive the 19-joint model through the
        # piecewise tendon map.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             remappings=[('robot_description', 'motor_description'),
                         ('joint_states', 'motor_commands')],
             condition=IfCondition(coupling_demo)),
        Node(package='rh8d_description', executable='rh8d_coupling_node.py',
             parameters=[{'prefix': PythonExpression(["'l_' if '", side, "' == 'left' else 'r_'"]),
                          'couple_ring_little': LaunchConfiguration('couple_ring_little'),
                          'output': 'joint_states'}],
             condition=IfCondition(coupling_demo)),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('rh8d_description'),
                                                    'rviz', 'rh8d.rviz'])]),
    ])
