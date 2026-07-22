# Integrating seed_rh8d_description into a full robot

Checklist and gotchas for embedding the RH8D in a larger robot description /
bringup. Everything here was learned the hard way during standalone
development — read before wiring things up.

## Copying this package into your repo

Copy the folder **without its `.git/`** (or use `git subtree add` if you want
to keep the history). The folder is a complete ament package; its location in
the repo doesn't matter, colcon finds it by `package.xml`. The `reference/`
folder (original NICOL URDFs) and `.vscode/` are not installed and can be
dropped if you want a leaner tree.

## Mounting the hand on your robot

Do NOT use `urdf/rh8d.urdf.xacro` (that's the standalone file — it creates its
own base/world links and the gz_ros2_control plugin). Instead:

```xml
<xacro:include filename="$(find seed_rh8d_description)/urdf/rh8d_macro.xacro"/>
<xacro:include filename="$(find seed_rh8d_description)/urdf/rh8d.gazebo.xacro"/>
<xacro:include filename="$(find seed_rh8d_description)/urdf/rh8d.ros2_control.xacro"/>

<xacro:rh8d prefix="r_" side="right" parent="your_wrist_flange"
            finger_coupling="mimic" couple_ring_little="true">
  <origin xyz="0 0 0.01" rpy="0 0 0"/>
</xacro:rh8d>

<!-- simulation only -->
<xacro:rh8d_gazebo prefix="r_" side="right"/>

<!-- if the hand joints live in your robot's ros2_control system, list them
     there instead of instantiating this (one <ros2_control> per hardware
     unit; mimic followers must be declared state-only, see
     rh8d.ros2_control.xacro for the exact per-mode joint sets) -->
<xacro:rh8d_ros2_control name="rh8d_right" prefix="r_"
                         finger_coupling="mimic" use_gazebo="true"/>
```

Both hands in one robot: instantiate twice with `l_`/`r_` prefixes — link,
joint, and sensor names are fully prefixed and won't collide.

## Gazebo checklist

- **Physics engine: dartsim only** (the gz default). bullet-featherstone
  corrupts existing articulations whenever a model is inserted at runtime
  (verified: 100 rad/s thrashing from a sphere spawned 0.5 m away) and does
  not enforce joint velocity limits. Your world also needs the `Sensors`
  (with ogre2) and `ForceTorque` systems — see `seed_rh8d_gazebo`'s
  `worlds/rh8d_world.sdf`.
- **One gz_ros2_control plugin per model.** The parent robot's plugin block
  serves the hand too; merge the hand's controller parameters (see
  `seed_rh8d_gazebo`'s `config/rh8d_controllers_*.yaml` for the per-mode
  joint sets) into your
  robot's controllers YAML. Keep `position_proportional_gain` ≥ 0.3 or the
  hand will feel sluggish regardless of joint velocity limits (velocity
  command = gain × error × update_rate).
- **Massless-root trap**: sdformat drops massless root links. The standalone
  file anchors world→base_link with a small mass; your robot presumably has
  a proper base already.
- **Mesh resolution**: append the package share's *parent* directory to
  `GZ_SIM_RESOURCE_PATH` (sdformat rewrites `package://` to `model://`).
- **Bridge config**: `seed_rh8d_gazebo`'s `config/gz_bridge_*.yaml` hardcodes
  the world name
  (`rh8d_world`) and model name (`rh8d_left`/`rh8d_right`) in the
  joint-state bridge entry — adapt both to your world/model names. The
  /clock bridge must stay in the YAML: parameter_bridge ignores
  `config_file` when any positional topic argument is given.
- Reuse `palm_ir_adapter.py` from `seed_rh8d_gazebo` (LaserScan → Range,
  0.255 m no-echo sentinel) and, for independent mode, this package's
  `rh8d_coupling_node.py` (set `prefix`).

## The dartsim limit-pinning rule (IMPORTANT)

A joint parked **exactly on** a position limit gets pinned by dartsim's limit
constraint and stays stuck until physically knocked free. Two protections are
built in: rest poses sit strictly inside the range (`phalanx_lower = -0.05`),
and the shipped command paths (coupling node, GUI bridge) clamp targets
`limit_margin` (0.03 rad) inside the limits.

**Anything that publishes trajectories directly to the hand's controller
(MoveIt, grasp planners, teleop) bypasses this** — clamp your targets the
same way, or a goal ending exactly at a limit can freeze that joint in sim.

## Tuning knobs (current values are sim-snappy, not hardware-calibrated)

| What | Where | Value |
|---|---|---|
| Finger speed/strength | `phalanx_velocity` / `phalanx_effort`, urdf/rh8d_finger.xacro | 8 rad/s / 30 |
| Limit slack / margin | `phalanx_lower` (xacro), `limit_margin` (nodes) | −0.05 / 0.03 |
| Tracking aggressiveness | `position_proportional_gain` (gz plugin block) | 0.3 |
| Contact friction | `mu` arg of `rh8d_link_friction`, urdf/rh8d.gazebo.xacro | 1.2 / pads 1.5 |
| Adaptive wrap sensitivity | `blocked_tolerance` / `blocked_velocity` (coupling node) | 0.12 / 0.05 |

Real-hardware calibration of velocity, wrap thresholds, and the mimic ratios
is still open. Full close currently takes ~0.4 s in sim — likely faster than
the real tendons.

## After changing any xacro

Regenerate the committed plain URDFs (used by Isaac Sim import and the
VSCode URDF visualizer, which can't evaluate the macros' Python expressions):

```bash
xacro urdf/rh8d.urdf.xacro side:=left use_ros2_control:=false > urdf/rh8d_left.urdf
xacro urdf/rh8d.urdf.xacro side:=right use_ros2_control:=false > urdf/rh8d_right.urdf
```

And run `test/validate_urdf.sh` (or `colcon test`) — it checks every
side × coupling × gazebo combination with check_urdf.

## Isaac Sim (untested)

Import the plain URDFs. `mimic` mode relies on the importer mapping `<mimic>`
tags to PhysX mimic constraints (Isaac ≥ 4.x — verify against your version);
`independent` mode is the base for PhysX tendon setups and works with the
coupling node unchanged.
