# rh8d_description

ROS 2 (Jazzy) description package for the [Seed Robotics RH8D](https://www.seedrobotics.com/rh8d-adult-robot-hand)
dexterous robot hand — left and right, with ros2_control and Gazebo (gz-sim
Harmonic) support.

The geometry originates from the [NICOL robot](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/research/neurobotics/nicol.html)
description by Nicolas Frick (MIT license, attribution preserved — see
[LICENSE](LICENSE)); it has been refactored from two 1600-line generated ROS 1
URDFs into parameterized xacro macros.

## Quick start

```bash
colcon build --packages-select rh8d_description
source install/setup.bash
ros2 launch rh8d_description display.launch.py side:=left finger_coupling:=mimic
```

`urdf/rh8d_left.urdf` and `urdf/rh8d_right.urdf` are pre-generated plain URDFs
(default arguments, no ros2_control block) for tools that don't run xacro,
e.g. the Isaac Sim URDF importer or the VSCode URDF visualizer. Regenerate
them after changing the xacros:

```bash
xacro urdf/rh8d.urdf.xacro side:=left use_ros2_control:=false > urdf/rh8d_left.urdf
xacro urdf/rh8d.urdf.xacro side:=right use_ros2_control:=false > urdf/rh8d_right.urdf
```

## Xacro arguments (`urdf/rh8d.urdf.xacro`)

| Argument | Default | Description |
|---|---|---|
| `side` | `left` | `left` / `right`; selects meshes, mirroring, thumb calibration |
| `finger_coupling` | `mimic` | `mimic` / `sequential` / `independent`, see below |
| `couple_ring_little` | `true` | ring + little finger share one actuator (as on the real hand) |
| `use_gazebo` | `false` | emit gz-sim sensors, contact params and the gz_ros2_control plugin |
| `use_ros2_control` | `true` | emit the `<ros2_control>` system block |
| `hardware_plugin` | `mock_components/GenericSystem` | hardware interface for real hardware |
| `controllers_file` | `config/rh8d_controllers_<side>.yaml` | parameters for gz_ros2_control |

To mount the hand on a robot, include `urdf/rh8d_macro.xacro` and instantiate
the `rh8d` macro with your own `parent` link and origin instead of using the
top-level file.

## Joint layout

The real RH8D has 8 actuators. Actuated joints (prefix `l_`/`r_`):

| Actuator | Joint(s) |
|---|---|
| Wrist rotation | `*_wrist_rotation_joint` |
| Wrist adduction | `*_wrist_adduction_joint` |
| Wrist flexion | `*_wrist_flexion_joint` |
| Thumb abduction | `*_thumb_abduction_joint` |
| Thumb flexion | `*_thumb_{proximal,medial,distal}_joint` |
| Index flexion | `*_index_{proximal,medial,distal}_joint` |
| Middle flexion | `*_middle_{proximal,medial,distal}_joint` |
| Ring+little flexion | `*_{ring,little}_{proximal,medial,distal}_joint` |

Fixed frames: `*_tool0` (grasp frame), `*_palm_ir` (palm distance sensor),
`*_<finger>_fingertip` (force sensor pads).

## Finger coupling modes

The phalanges of each finger are tendon-driven by a single motor. Three ways
to model this:

- **`mimic`** (default): medial and distal phalanges follow the proximal
  joint linearly, scaled so each reaches its limit together
  (`follower = leader × upper_follower/upper_leader`). 8-DOF command
  interface identical to the real hand. Works with every standard tool
  (joint_state_publisher, ros2_control, gz-sim, Isaac Sim ≥ 4.x mimic import).
- **`sequential`**: closer to the real closing behavior — phalanges engage
  *one after another*. A virtual motor joint per tendon
  (`*_thumb_flexion_joint`, `*_index_flexion_joint`, `*_middle_flexion_joint`,
  `*_ring_little_flexion_joint`) is the mimic leader; phalanges follow with
  staggered negative offsets and are held at their limits through their dead
  zone by the physics engine. Notes:
  - Follower joints are deliberately **not** listed in the `<ros2_control>`
    block — gz_ros2_control's own mimic handling ignores offsets (bug present
    up to at least Jazzy); the constraints are left to the physics engine
    (URDF `<mimic>` including offsets is converted to SDF by sdformat ≥ 14).
  - For RViz, `display.launch.py` pipes the joint_state_publisher GUI through
    `scripts/mimic_joint_clamper.py`, which applies the limit clamping that
    stock joint_state_publisher does not. Verified working.
  - **Gazebo status (Harmonic, verified empirically): imperfect.** dartsim
    (the default engine) drops mimic constraints entirely; the shipped world
    therefore selects bullet-featherstone, which does enforce them — the
    drive and proximal/medial sequencing behave correctly, but the limit
    clamping loses against the stacked constraint on the distal phalanx
    (it is dragged below its lower limit) and the drive can stall against
    the constraint fight. For physically accurate sequential closing in
    Gazebo, use `independent` mode with a coupling controller (planned) or
    `mimic` mode. In Isaac Sim, PhysX mimic constraints support offsets
    natively and are the intended path for this mode (verify against your
    Isaac version).
- **`independent`**: all hand joints actuated, no mimic tags. Combined with
  the shipped **tendon coupling controller**
  (`scripts/rh8d_coupling_node.py`) this is the most faithful mode: one
  command per real motor, physically correct sequential closing (verified in
  Gazebo: proximal → medial → distal, each engaging only after the previous
  saturates), and with `adaptive:=true` a contact-aware wrap — when a phalanx
  is blocked by an object (large tracking error at near-zero velocity), the
  remaining motor travel flows to the more distal joints, like the real
  tendon. This is the behavior the mimic-based modes structurally cannot
  express.

### Tendon coupling controller (independent mode)

`rh8d_coupling_node.py` listens on `motor_commands` (sensor_msgs/JointState,
one entry per motor axis — same names as the `sequential` drive joints plus
the wrist/abduction pass-throughs) and outputs either a `JointTrajectory` for
the `hand_controller` (simulation / hardware) or `joint_states` directly
(RViz demo). Parameters: `prefix`, `couple_ring_little`, `adaptive`,
`blocked_tolerance` (rad), `blocked_velocity` (rad/s), `rate`, `output`.

- Gazebo: started automatically by `gazebo.launch.py` when
  `finger_coupling:=independent` (disable with `use_coupling:=false`;
  adaptive wrap on by default, `adaptive:=false` for strict sequencing).
  Command e.g. `ros2 topic pub /motor_commands sensor_msgs/msg/JointState
  "{name: [l_index_flexion_joint], position: [2.0]}"`.
- RViz: `display.launch.py finger_coupling:=independent` shows one slider
  per motor (the GUI is fed the sequential variant's description) driving
  the full 19-joint model through the coupling map.

## Gazebo simulation

Requires `ros-jazzy-ros-gz` and `ros-jazzy-gz-ros2-control` (intentionally not
hard dependencies of this package):

```bash
ros2 launch rh8d_description gazebo.launch.py side:=left finger_coupling:=mimic
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

**Physics engine note:** only `finger_coupling:=sequential` uses
`worlds/rh8d_world_bullet.sdf` (bullet-featherstone, the sole engine with
native mimic-offset constraints). That engine is empirically unstable —
runtime model insertion (e.g. dropping a shape in from the GUI) can make
existing articulated models thrash violently, entirely without contact —
so treat sequential-in-Gazebo as **deprecated**: the engine also oscillates
against the joint limits even without any object present. For sequential
closing behavior in simulation always use `finger_coupling:=independent`
with the coupling controller (dartsim, fully stable, verified) — it produces
the same one-after-another engagement with correct limits plus the adaptive
wrap. Sequential mode's real targets are RViz and Isaac Sim.

`motor_gui:=true` works in every mode: in independent(+coupling) mode the
sliders are the 8 motor axes feeding the coupling node; in mimic/sequential
(and raw independent) the sliders are the controller's actuated joints,
routed through `joint_gui_to_trajectory.py`, which learns the joint set from
the controller automatically.

## Sensors (Gazebo, `use_gazebo:=true`)

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
  -p config_file:=$(ros2 pkg prefix --share rh8d_description)/config/gz_bridge_left.yaml
```

Simulation requires `gz_ros2_control`, `ros_gz_sim` and `ros_gz_bridge`
(intentionally not hard dependencies of this package).

## Tests

`colcon test` expands the xacro for all side × coupling combinations
(plus the Gazebo variants) and runs `check_urdf` on the result
(`test/validate_urdf.sh`, also runnable standalone).

## Changes vs. the original NICOL URDFs

Deliberate fixes, in addition to the restructuring:

- left forearm collision mesh scale corrected (`1 1 1` → `0.001 0.001 0.001`)
- right `jointR3` upper limit outlier unified (`1.57` → `1.0`)
- stray/corrupt XML fragments removed
- sensor pad ("bumper") joints are `fixed` on both sides (the zero-limit
  revolute variant was a Gazebo Classic anti-lumping workaround; gz-sim uses
  `<preserveFixedJoint>`); all five pads share one uniform inertial
- high-res meshes for palm/tips/pads visuals on both sides, low-res for
  collisions; unified `package://rh8d_description` mesh paths
- ROS 1 remnants removed (`<transmission>`, `libgazebo_ros_control`,
  NICOL laser/F3D plugins → replaced by ros2_control + gz-sim sensors)
- world link and fake `joint_base` removed; the hand mounts via the macro's
  `parent` parameter
