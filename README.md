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
  - In Gazebo the mimic constraints are enforced natively by the physics
    engine (URDF `<mimic>` including offsets is converted to SDF by
    sdformat ≥ 14). Follower joints are deliberately **not** listed in the
    `<ros2_control>` block — gz_ros2_control's own mimic handling ignores
    offsets (bug present up to at least Jazzy).
  - For RViz, `display.launch.py` pipes the joint_state_publisher GUI through
    `scripts/mimic_joint_clamper.py`, which applies the limit clamping that
    stock joint_state_publisher does not.
- **`independent`**: all 21 hand joints actuated (plus wrist), no coupling.
  For grasping research where contact should interrupt the tendon coupling,
  or as a base for PhysX tendon setups in Isaac Sim.

None of the mimic modes can represent the adaptive wrap of a real tendon
(distal phalanges continuing to close when a proximal one is blocked); use
`independent` plus your own coupling controller or PhysX tendons for that.

## Sensors (Gazebo, `use_gazebo:=true`)

- **Fingertip 3D force sensors**: one `force_torque` sensor per fingertip pad
  joint, publishing on gz topics `rh8d/<side>/fingertip/<finger>/wrench`
  (z = normal force, x/y = shear, in the pad frame).
- **Palm IR distance sensor**: `gpu_lidar` (3–254 mm) on
  `rh8d/<side>/palm_ir/scan`.

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
