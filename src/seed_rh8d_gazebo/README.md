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
  (x = normal force — the pad's thickness axis in the fingertip frame —
  y/z = shear; verified against the real sensors).
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

## Driver-native interface

`scripts/driver_interface.py` puts the **driver's native tick interface on
the simulation**, so code written against the hardware — the user samples in
`seed_hand_driver/user_samples/`, or existing tick-based scripts — runs
against Gazebo unchanged. It is the mirror of `seed_hand_driver`'s
`aligned_interface.py`, which does the same in the other direction (sim
topics and units on the real hand). `gazebo.launch.py` starts it by default;
`driver_interface:=false` skips it.

| Topic | Type | Direction |
|---|---|---|
| `<L_\|R_>Joints` | `seed_hand_msgs/AllJoints` | published, at `frequency` (50 Hz) |
| `<L_\|R_>Main_Boards` | `seed_hand_msgs/AllMainBoards` | published |
| `<L_\|R_>speed_position` | `seed_hand_msgs/JointListSetSpeedPos` | subscribed |
| `<L_\|R_>stiffness` | `seed_hand_msgs/JointListSetStiffness` | subscribed |
| `<L_\|R_>clear_error` | `seed_hand_msgs/ClearHWError` | subscribed |
| `<L_\|R_>shutdown_condition` | `seed_hand_msgs/SetShutdownCond` | subscribed |

Motor names and bus IDs match `seed_hand_driver/config/RH8D_<L|R>.yaml`
(`r_index_flexion_joint` = 36, …; IDs run `base_id`…`base_id`+8 with the main
board first). Joints are addressable by name or by bus ID as a numeric
string, as on the hardware.

```bash
ros2 launch seed_rh8d_gazebo gazebo.launch.py side:=right
python3 src/seed_hand_driver/user_samples/user_sample_2_set_speed_position_R.py
```

### Units and calibration

Internally the node works in the same aligned units as `aligned_interface`
and `rh8d_coupling_node` (finger flexion = closure fraction 0–1, wrist and
thumb adduction = radians) and converts to ticks with the **same parameters**:
`calib.<axis_without_prefix>.tick_min` / `tick_max`, where `tick_min` is the
axis minimum (fingers open, wrist lower limit). One calibration block
therefore serves both interfaces. The defaults (0–4095 over the full model
range) are **not hardware-calibrated**.

`target_speed` (0–1023, **0 = maximum**, `-1` = keep previous) is honoured by
rate-limiting the command internally, the way the servo's profile velocity
does — a single trajectory point cannot carry a per-joint speed.
`speed_scale` (device units per aligned-unit/s) sets the conversion, matching
`aligned_interface`'s parameter of the same name.

In `independent` mode the node feeds the coupling node's motor axes on
`motor_commands`, so ticks drive the full sequential proximal → medial →
distal wrap; in `mimic` mode it commands the trajectory controller's leader
joints directly.

### What is faithful, and what is modelled

- **Faithful**: positions, target/present position and speed, `moving`,
  per-motor speed profiles, name and bus-ID addressing, the driver's
  validation and rate-limiting (stiffness range 1–9, 30 s between two
  `stiffness` or `clear_error` writes on one joint), `palm_ir_sensor` in mm
  with **255 = nothing in range**.
- **Modelled**: `current` (and the `stress_level` derived from it) comes from
  the position tracking error, the way a position servo's torque does — gz
  reports a near-zero joint force for position-controlled joints, so the
  error is the usable load signal. It stays near zero in free motion and
  grows as soon as a finger is blocked, which is what current-watching grasp
  code (`user_sample_7`) looks for. Tune `current_per_error` and
  `current_limit`.
- **Not simulated**: `stiffness` is recorded and echoed back but has no
  mechanical effect (the sim uses the controller gains);
  `hw_error_condition` is always 0, `temperature` and `torque_limit` are
  constants, and the palm capacitive sensors report 0.
- The wrist and thumb-abduction axes stop `limit_margin` (0.03 rad) short of
  their limits — the dartsim limit-pinning protection — so their extreme
  ticks read back a couple of percent inside the range. The flexion axes
  round-trip exactly, since the coupling node's closure of 1.0 already means
  "margin-limited full closure".
- The driver's `RL_` two-hands-on-one-bus mode has no sim equivalent; run one
  simulation and one interface per hand.

The node stays silent until the first `speed_position` command arrives, so it
doesn't fight `motor_gui:=true` or a trajectory publisher on startup. After
that it commands all 8 motors every cycle (holding uncommanded ones where
they are), exactly as the hardware bus does.

## Tests

`test/` holds the integration tests for this package. `colcon test` runs the
three that need no simulation (`driver_interface` against faked joint states,
the `aligned_interface` round trip, and every `gazebo.launch.py` argument
combination). The simulation tests are opt-in because each starts gz-sim:

```bash
colcon build --cmake-args -DSEED_SIM_TESTS=ON
colcon test --packages-select seed_rh8d_gazebo
```

| Test | What it covers |
|---|---|
| `test_driver_interface.py` | tick conversion, name and bus-ID addressing, speed profiles, the current model, validation and rate limits, publish rate |
| `test_interface_parity.py` | `motor_commands → aligned_interface → ticks → driver_interface → motor_commands` is an identity, on default and overridden calibration |
| `test_launch_files.py` | all 64 `gazebo.launch.py` argument combinations resolve |
| `test_sim_driver_interface.py` | commands move the simulated hand and read back correctly (per side and coupling mode) |
| `test_sim_joint_limits.py` | **regression**: a large step must not leave a joint pinned on its limit |
| `test_sim_user_samples.py` | the `seed_hand_driver` user samples, unmodified, including the sample 7 grab |

The sim tests take a side and coupling mode, so a failure is quick to
reproduce on its own:

```bash
python3 src/seed_rh8d_gazebo/test/test_sim_driver_interface.py independent left
```

## Controller configs

`config/rh8d_controllers_<side>[_independent].yaml` — the actuated joint set
differs per coupling mode; `gazebo.launch.py` picks the right file and passes
it to the description's `controllers_file` xacro arg. When embedding the hand
in a larger robot, merge these parameters into your robot's controllers YAML
instead (see INTEGRATION.md).
