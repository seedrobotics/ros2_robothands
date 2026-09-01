from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration('side')
    coupling = LaunchConfiguration('finger_coupling')
    desc_pkg = FindPackageShare('seed_rh8d_description')
    sim_pkg = FindPackageShare('seed_rh8d_gazebo')

    # rh8d_controllers_<side>[_independent].yaml - the actuated joint set
    # differs per coupling mode.
    controllers_file = PathJoinSubstitution([sim_pkg, 'config', PythonExpression([
        "'rh8d_controllers_' + '", side, "'",
        " + ('_independent' if '", coupling, "' == 'independent' else '')",
        " + '.yaml'"])])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([desc_pkg, 'urdf', 'rh8d.urdf.xacro']),
            ' use_gazebo:=true',
            ' side:=', side,
            ' finger_coupling:=', coupling,
            ' couple_ring_little:=', LaunchConfiguration('couple_ring_little'),
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str)

    world = PathJoinSubstitution([sim_pkg, 'worlds', 'rh8d_world.sdf'])
    gz_args = PythonExpression(
        ["('-s ' if '", LaunchConfiguration('headless'), "' == 'true' else '') + '-r -v1 '"])

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('finger_coupling', default_value='mimic',
                              choices=['mimic', 'independent']),
        DeclareLaunchArgument('couple_ring_little', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_coupling', default_value='true',
                              description='Run the tendon coupling controller '
                                          '(independent mode only)'),
        DeclareLaunchArgument('adaptive', default_value='true',
                              description='Contact-adaptive finger wrap in the '
                                          'coupling controller'),
        DeclareLaunchArgument('motor_gui', default_value='false',
                              description='Slider panel (one per real motor) '
                                          'commanding the sim via the coupling '
                                          'controller (independent mode)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('driver_interface', default_value='true',
                              description='Expose the driver\'s native '
                                          'tick interface (<L_|R_>Joints, '
                                          '<L_|R_>speed_position, ...) on '
                                          'the simulation'),

        # Lets gz resolve the model://seed_rh8d_description/... mesh URIs that
        # sdformat generates from the package:// paths.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  PathJoinSubstitution([desc_pkg, '..'])),

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
                 [sim_pkg, 'config', PythonExpression(["'gz_bridge_' + '", side, "' + '.yaml'"])])}]),

        # Real-sensor semantics for the palm IR: single Range value, 0.255 m
        # when nothing is in range (the hardware reports 255, never inf).
        Node(package='seed_rh8d_gazebo', executable='palm_ir_adapter.py',
             remappings=[('scan', ['/rh8d/', side, '/palm_ir/scan']),
                         ('range', ['/rh8d/', side, '/palm_ir/range'])],
             parameters=[{'use_sim_time': True}]),

        # Tendon coupling controller: maps the 8 motor axes onto the 19
        # independent joints with sequential engagement (and optional
        # contact-adaptive wrap). Only meaningful in independent mode.
        # It also latches the generated motor panel description on
        # motor_description for the motor slider GUI.
        Node(package='seed_rh8d_description', executable='rh8d_coupling_node.py',
             condition=IfCondition(PythonExpression(
                 ["'", coupling, "' == 'independent' and '",
                  LaunchConfiguration('use_coupling'), "' == 'true'"])),
             parameters=[{'prefix': PythonExpression(["'l_' if '", side, "' == 'left' else 'r_'"]),
                          'couple_ring_little': LaunchConfiguration('couple_ring_little'),
                          'adaptive': LaunchConfiguration('adaptive'),
                          'output': 'trajectory',
                          'use_sim_time': True}]),

        # motor_gui:=true - one slider per real motor, driving the physics
        # sim through the coupling controller.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             remappings=[('robot_description', 'motor_description'),
                         ('joint_states', 'motor_commands')],
             condition=IfCondition(PythonExpression(
                 ["'", coupling, "' == 'independent' and '",
                  LaunchConfiguration('use_coupling'), "' == 'true' and '",
                  LaunchConfiguration('motor_gui'), "' == 'true'"])),
             parameters=[{'use_sim_time': True}]),

        # motor_gui in the other modes (mimic, or independent without the
        # coupling node): sliders for the actuated joints, sent
        # to the trajectory controller via the gui-to-trajectory bridge,
        # which learns the controller's joint set from controller_state.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             remappings=[('joint_states', 'gui_joint_states')],
             condition=IfCondition(PythonExpression(
                 ["('", coupling, "' != 'independent' or '",
                  LaunchConfiguration('use_coupling'), "' != 'true') and '",
                  LaunchConfiguration('motor_gui'), "' == 'true'"])),
             parameters=[{'use_sim_time': True}]),
        Node(package='seed_rh8d_gazebo', executable='joint_gui_to_trajectory.py',
             condition=IfCondition(PythonExpression(
                 ["('", coupling, "' != 'independent' or '",
                  LaunchConfiguration('use_coupling'), "' != 'true') and '",
                  LaunchConfiguration('motor_gui'), "' == 'true'"])),
             parameters=[{'use_sim_time': True}]),

        # Driver-native interface: the same topics, message types, motor
        # names and tick units as seed_hand_driver, so hardware code runs
        # unchanged against the sim. The mirror of the driver's
        # aligned_interface, which does the same in the other direction.
        # In independent mode it feeds the coupling node's motor axes; in
        # mimic mode it commands the trajectory controller directly.
        Node(package='seed_rh8d_gazebo', executable='driver_interface.py',
             name='hand_driver_interface',
             output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('driver_interface'), "' == 'true' and ",
                  "('", coupling, "' != 'independent' or '",
                  LaunchConfiguration('use_coupling'), "' == 'true')"])),
             parameters=[{
                 'topic_prefix': PythonExpression(
                     ["'L_' if '", side, "' == 'left' else 'R_'"]),
                 'joint_prefix': PythonExpression(
                     ["'l_' if '", side, "' == 'left' else 'r_'"]),
                 'base_id': PythonExpression(
                     ["40 if '", side, "' == 'left' else 30"]),
                 'finger_coupling': coupling,
                 'couple_ring_little': LaunchConfiguration('couple_ring_little'),
                 'use_sim_time': True}],
             remappings=[('palm_ir/range', ['/rh8d/', side, '/palm_ir/range'])]),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([desc_pkg, 'rviz', 'rh8d.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
