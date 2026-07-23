"""Bring up a Seed Robotics hand: driver, optional sensors, RViz, motor GUI.

    ros2 launch seed_hand_bringup hand.launch.py side:=right
    ros2 launch seed_hand_bringup hand.launch.py side:=right use_sensors:=true
    ros2 launch seed_hand_bringup hand.launch.py side:=right rviz:=true motor_gui:=true

rviz:=true shows the hand's measured pose live (motor states mapped through
the tendon coupling model); motor_gui:=true adds the same motor slider panel
as the sim's gazebo.launch.py (fingers 0..1 closure, wrist in radians),
commanding the real hand via motor_commands. CAUTION: the sliders start at
0/centered and command that pose as soon as the panel opens.

side:=both drives two hands on one serial port (see RH8D_RL.yaml) and, with
use_sensors:=true, starts one sensor node per hand (rviz/motor_gui support
single sides only). Serial ports must be set in the driver and sensor YAML
configs first. For visualization on a separate machine use view.launch.py.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HAND_CONFIG = {'left': 'RH8D_L.yaml', 'right': 'RH8D_R.yaml', 'both': 'RH8D_RL.yaml'}


def setup(context, *args, **kwargs):
    cfg = context.launch_configurations
    side = cfg['side']
    use_sensors = cfg['use_sensors'].lower() == 'true'
    aligned = cfg['aligned_interface'].lower() == 'true'
    rviz = cfg['rviz'].lower() == 'true'
    motor_gui = cfg['motor_gui'].lower() == 'true'

    actions = [
        Node(
            package='seed_hand_driver',
            executable='hand_handle_node',
            name='hand_handle_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('seed_hand_driver'), 'config', HAND_CONFIG[side]])],
        ),
    ]

    if aligned:
        # sim-compatible topics/units next to the native tick interface;
        # side:=both shares the RL_ driver but needs one bridge per hand
        topic_prefix = {'left': 'L_', 'right': 'R_', 'both': 'RL_'}[side]
        for s in (['left', 'right'] if side == 'both' else [side]):
            actions.append(Node(
                package='seed_hand_driver',
                executable='aligned_interface',
                name=f'hand_aligned_interface_{s}',
                output='screen',
                parameters=[{'topic_prefix': topic_prefix,
                             'joint_prefix': s[0] + '_'}],
            ))

    if (rviz or motor_gui) and side == 'both':
        actions.append(LogInfo(
            msg='rviz/motor_gui support side:=left|right only — skipped '
                '(the standalone model can show one hand per instance)'))
    elif rviz or motor_gui:
        if not aligned:
            actions.append(LogInfo(
                msg='rviz/motor_gui need the aligned interface — the view '
                    'will be static and sliders inert with '
                    'aligned_interface:=false'))
        desc_pkg = FindPackageShare('seed_rh8d_description')
        robot_description = ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([desc_pkg, 'urdf', 'rh8d.urdf.xacro']),
                ' side:=', side,
                ' use_ros2_control:=false',
            ]),
            value_type=str)

        # measured motor axes -> full joint state through the tendon model;
        # also latches the motor panel description for the slider GUI
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description}]),
            Node(package='seed_rh8d_description', executable='rh8d_coupling_node.py',
                 name='motor_state_mapper',
                 remappings=[('motor_commands', 'motor_states')],
                 parameters=[{'prefix': side[0] + '_',
                              'output': 'joint_states'}]),
        ]
        if rviz:
            actions.append(Node(
                package='rviz2', executable='rviz2',
                arguments=['-d', PathJoinSubstitution(
                    [desc_pkg, 'rviz', 'rh8d.rviz'])]))
        if motor_gui:
            actions.append(Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                remappings=[('robot_description', 'motor_description'),
                            ('joint_states', 'motor_commands')]))

    if use_sensors:
        sensor_sides = ['left', 'right'] if side == 'both' else [side]
        for s in sensor_sides:
            actions.append(IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('ros2_sensor_pkg'), 'launch', 'Sensors.launch.py']),
                launch_arguments={
                    'node_name': f'seed_fts3_{s}',
                    'config_file': PathJoinSubstitution([
                        FindPackageShare('ros2_sensor_pkg'), 'config', f'sensors_{s}.yaml']),
                }.items(),
            ))
            # per-finger WrenchStamped on the sim's topic names.
            # axis_map determined empirically on the right hand (arrow points
            # with the applied force, matching the sim's child_to_parent
            # convention); assumed identical mounting on the left.
            actions.append(Node(
                package='seed_hand_bringup',
                executable='sensor_wrench_adapter',
                name=f'sensor_wrench_adapter_{s}',
                output='screen',
                parameters=[{'side': s,
                             'axis_map': ['z', '-y', 'x']}],
            ))

    if cfg['plot'].lower() == 'true':
        actions.append(Node(
            package='plotjuggler', executable='plotjuggler',
            arguments=['-n', '-l', PathJoinSubstitution([
                FindPackageShare('seed_hand_bringup'), 'config', 'fingertips.xml'])],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right',
                              choices=['left', 'right', 'both'],
                              description='Which hand(s) to bring up'),
        DeclareLaunchArgument('use_sensors', default_value='false',
                              description='Also start the in-hand FTS sensor node(s)'),
        DeclareLaunchArgument('aligned_interface', default_value='true',
                              description='Start the sim-aligned interface '
                                          '(motor_commands / motor_states / '
                                          'hand_controller/joint_trajectory)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Show the measured hand pose live in RViz'),
        DeclareLaunchArgument('motor_gui', default_value='false',
                              description='Motor slider panel commanding the real '
                                          'hand (same panel as the sim)'),
        DeclareLaunchArgument('plot', default_value='false',
                              description='Open PlotJuggler with the fingertip '
                                          'sensor layout'),
        OpaqueFunction(function=setup),
    ])
