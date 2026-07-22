# seed_rh8d_gazebo

Gazebo (gz-sim Harmonic) simulation of the Seed Robotics RH8D hand. The robot
model itself lives in [seed_rh8d_description](../seed_rh8d_description/) —
this package adds the world, the gz_ros2_control controller configs, the gz
bridge configs, and the sim helper nodes.

Requires `ros-jazzy-ros-gz` and `ros-jazzy-gz-ros2-control`.

## Quick start

```bash
ros2 launch seed_rh8d_gazebo gazebo.launch.py side:=left finger_coupling:=mimic
# headless server only: headless:=true
```

This starts gz-sim Harmonic with `worlds/rh8d_world.sdf` (dartsim, the
default engine — mimic-mode followers are enforced by gz_ros2_control, so
no SDF mimic constraints are needed), spawns the hand anchored to the world,
activates `joint_state_broadcaster` + `hand_controller` (a
JointTrajectoryController over the actuated joints), and bridges clock,
sensors and the complete gz joint state (including mimic followers, so RViz
shows full TF) to ROS. Command the hand via
`/hand_controller/joint_trajectory`.

Launch arguments: `side`, `finger_coupling`, `couple_ring_little`,
`headless`, `use_coupling`, `adaptive`, `motor_gui`, `rviz` — see
[seed_rh8d_description's README](../seed_rh8d_description/README.md) for the
coupling modes and the tendon coupling controller they configure.

**Physics engine note:** the world runs dartsim (Gazebo's default engine).
It does not support SDF mimic constraints, so mimic-mode follower joints are
declared state-only in the ros2_control block and enforced by gz_ros2_control.
Do not switch to bullet-featherstone — see
[INTEGRATION.md](../seed_rh8d_description/INTEGRATION.md).

`motor_gui:=true` works in both modes: in independent(+coupling) mode the
sliders are the 8 motor axes feeding the coupling node; in mimic mode (and
raw independent) the sliders are the controller's actuated joints, routed
through `joint_gui_to_trajectory.py`, which learns the joint set from the
controller automatically.

## Sensors

- **Fingertip 3D force sensors**: one `force_torque` sensor per fingertip pad
  joint, publishing on gz topics `rh8d/<side>/fingertip/<finger>/wrench`
  (z = normal force, x/y = shear, in the pad frame).
- **Palm IR distance sensor**: `gpu_lidar` (3–254 mm) on
  `rh8d/<side>/palm_ir/scan`; `palm_ir_adapter.py` (started by
  `gazebo.launch.py`) converts it to a single `sensor_msgs/Range` on
  `/rh8d/<side>/palm_ir/range` with the real sensor's semantics: **0.255 m
  when nothing is in range** (the hardware reports 255, never infinity).

Bridge them to ROS with the shipped configs:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix --share seed_rh8d_gazebo)/config/gz_bridge_left.yaml
```

## Controller configs

`config/rh8d_controllers_<side>[_independent].yaml` — the actuated joint set
differs per coupling mode; `gazebo.launch.py` picks the right file and passes
it to the description's `controllers_file` xacro arg. When embedding the hand
in a larger robot, merge these parameters into your robot's controllers YAML
instead (see INTEGRATION.md).
