# rokoko_glove_control

Teleoperate a Seed Robotics RH8D from a Rokoko Smartglove. The node reads
solved hand poses from Rokoko's hand solver and publishes them on
`motor_commands`, the workspace's aligned interface — so the same node drives
the real hand and the Gazebo simulation.


## What has to be running

**Always: the Rokoko hand solver.** It turns raw sensor poses into the hand
skeleton this reads, and serves them on `127.0.0.1:12277`. Nothing works
without it, and it is not a ROS node — start it in its own terminal:

```bash
~/.local/share/rokoko-device-sdk/bin/rkk-hand-solver
```

Then **one** hand target, and the glove node.

### Against the simulation

```bash
ros2 launch seed_rh8d_gazebo gazebo.launch.py side:=left finger_coupling:=independent
ros2 launch rokoko_glove_control glove.launch.py side:=left
```

`finger_coupling:=independent` is **required**, not optional. It is what starts
`rh8d_coupling_node`, and that node is the only subscriber to
`motor_commands`. In the default `mimic` mode the glove node publishes into
the void and the hand sits still. Add `rviz:=true` for the RViz view alongside
the Gazebo window.

### Against the real hand

```bash
ros2 launch seed_hand_bringup hand.launch.py side:=left
ros2 launch rokoko_glove_control glove.launch.py side:=left
```

`hand.launch.py` starts `aligned_interface` by default, which is what
subscribes to `motor_commands` and converts it to the driver's ticks. No
coupling argument is needed on this path.

> **CAUTION**: the hand goes to wherever your hand is as soon as the glove is
> seen. `max_range_per_second` bounds how fast it gets there (a full axis range
> in a third of a second by default) and nothing is published until a pose
> actually arrives, but it still moves to your pose — so start with your hand
> somewhere sane, and mind the tick calibration warning in the root README.

### Both sides

`side:=` picks which glove hand to follow (`left`, `right`, or `auto` for
whichever appears first) and derives the robot joint prefix from it. Set
`joint_prefix:=` explicitly to drive a right hand from a left glove or vice
versa.

## Calibration

Two different calibrations are in play, and only one of them lives here.

**The glove's open/fist calibration** is this package's, and it is
load-bearing: without it a channel never enters its window, gets pinned at a
constant, and that joint stops moving.

```bash
ros2 run rokoko_glove_control glove_calibrate
```

Four poses, about a second each, Enter between them:

1. Hand flat and open, thumb straight and spread wide
2. Curl the four fingers into a fist, thumb still out and spread
3. Thumb straight, sweep it across the palm toward the little finger
4. Curl the thumb in toward the palm as far as it goes

It writes `rh8d_calibration.json` beside the glove modules, which is where the
node looks by default — no argument needed. The node logs which file it
loaded, or warns if it found none. `--host`, `--port` and `--calibration`
override the defaults.

Poses 3 and 4 exist because a fist adducts and flexes the thumb at once: a
thumb calibrated from the open/fist pair alone gets a flexion range so narrow
that its flexion channel tracks adduction instead. If the thumb misbehaves,
recalibrate before suspecting the mapping.

**The hand's tick calibration** is a separate thing entirely, shared with
every other command source — see *Aligned interface* in the root README.

## Launch arguments

| Argument | Default | |
|---|---|---|
| `side` | `right` | which glove hand: `left`, `right`, `auto` |
| `joint_prefix` | derived | robot prefix; defaults to matching `side` |
| `calibration_file` | auto | glove calibration JSON |
| `solver_host` | `127.0.0.1` | where rkk-hand-solver is |
| `rate` | `50.0` | publish rate, Hz |
| `lock_wrist` | `false` | centre the wrist, follow the fingers only |
| `wrist_scale` | `1.0` | multiply all three wrist angles |
| `max_range_per_second` | `3.0` | slew limit, axis ranges per second; `0` disables |

## What the node publishes

`motor_commands` (`sensor_msgs/JointState`), eight motor axes in aligned
units — finger flexion as a closure fraction 0 (open) to 1 (closed), wrist and
thumb adduction in radians:

```
<p>wrist_rotation_joint   <p>thumb_adduction_joint   <p>index_flexion_joint
<p>wrist_adduction_joint  <p>thumb_flexion_joint     <p>middle_flexion_joint
<p>wrist_flexion_joint                               <p>ring_little_flexion_joint
```

Ticks are produced downstream, so the glove path is calibrated in the same one
place as everything else.

## Things worth knowing

**Wrist rotation follows your forearm, not your wrist.** Pronation and
supination happen between the forearm bones, above the wrist, so the wrist
joint's own axial twist covers only a fraction of the travel — rotating your
arm 180° moves it maybe 45°. The node reads the forearm's roll instead, from
the world-referenced frame the solver streams alongside the hand. That value
is absolute and meaningless on its own, so it is **tared when the node
starts**: hold your forearm in a neutral pose as it comes up, and restart the
node to re-zero.

**Both wrist axes are inverted** against this model, measured on the hardware;
rotation and the thumb are not. The signs are `CHANNEL_SIGN` in
`scripts/glove_node.py`.

**The wrist pair is routed straight**, deliberately not through
`rh8d_mapping.classify_joint`. That function crosses flexion and adduction to
compensate for the demo config's labels, and it matches on substrings, so it
cannot tell `Wrist Adduction` from `r_wrist_adduction_joint`. The model's
joint names are anatomically accurate, so a crossing there would be a second
one. See `CHANNEL_FOR_AXIS`.

**A stale stream holds position.** The last pose keeps being published, so the
hand holds rather than relaxing or jumping. If the chosen hand disappears from
the solver entirely, the node stops publishing and the hand holds its last
target.

## Layout

```
scripts/            glove_node.py, glove_calibrate.py   the ros2 run entry points
rokoko_glove_control/
  hand_joints.py        skeleton, pose maths, the glove calibration
  rgmp_client.py        Rokoko's RGMP v2 wire protocol
  rgmp_client_rh8d.py   HandReader, and the standalone --check / --calibrate CLI
  rh8d_mapping.py       channel semantics, ranges, the thumb gate
launch/  test/
```

The four modules import each other flatly, so they install beside the scripts
in `lib/rokoko_glove_control/`. They are shared verbatim with the standalone
demo tool; the ROS-specific parts live in `scripts/`.

`rgmp_client_rh8d.py` also has a live wrist readout, useful for confirming a
sign convention:

```bash
python3 src/rokoko_glove_control/rokoko_glove_control/rgmp_client_rh8d.py --check
```

## Tests

```bash
colcon test --packages-select rokoko_glove_control
```

Synthetic poses, so it needs no glove and no solver. It covers the channel
routing, the crossed-wrist divergence from `rh8d_mapping`, the unit conversion
against the URDF joint limits, the forearm-roll unwrap and tare, the slew
limit, and that the axis ranges have not drifted from the model.
