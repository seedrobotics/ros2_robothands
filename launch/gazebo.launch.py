from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration('side')
    coupling = LaunchConfiguration('finger_coupling')
    pkg = FindPackageShare('rh8d_description')

    # rh8d_controllers_<side>.yaml, or rh8d_controllers_<side>_sequential.yaml
    # in sequential mode (different actuated joint names).
    controllers_file = PathJoinSubstitution([pkg, 'config', PythonExpression([
        "'rh8d_controllers_' + '", side, "'",
        " + ('_sequential' if '", coupling, "' == 'sequential' else '')",
        " + '.yaml'"])])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg, 'urdf', 'rh8d.urdf.xacro']),
            ' use_gazebo:=true',
            ' side:=', side,
            ' finger_coupling:=', coupling,
            ' couple_ring_little:=', LaunchConfiguration('couple_ring_little'),
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str)

    world = PathJoinSubstitution([pkg, 'worlds', 'rh8d_world.sdf'])
    gz_args = PythonExpression(
        ["('-s ' if '", LaunchConfiguration('headless'), "' == 'true' else '') + '-r -v1 '"])

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('finger_coupling', default_value='mimic',
                              choices=['mimic', 'sequential', 'independent']),
        DeclareLaunchArgument('couple_ring_little', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),

        # Lets gz resolve the model://rh8d_description/... mesh URIs that
        # sdformat generates from the package:// paths.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  PathJoinSubstitution([pkg, '..'])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': [gz_args, world]}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', ['rh8d_', side]],
             output='screen'),

        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'hand_controller',
                        '--controller-manager', '/controller_manager']),

        # Note: no positional topic arguments here — parameter_bridge ignores
        # config_file when any are present (the /clock bridge lives in the yaml).
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': PathJoinSubstitution(
                 [pkg, 'config', PythonExpression(["'gz_bridge_' + '", side, "' + '.yaml'"])])}]),
    ])
