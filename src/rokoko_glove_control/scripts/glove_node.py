#!/usr/bin/env python3
"""Rokoko Smartglove -> RH8D, over the workspace's aligned interface.

Reads solved hand poses straight from the Rokoko hand solver (RGMP on
127.0.0.1:12277) and publishes them as `motor_commands`, so the glove drives
the real hand and the Gazebo simulation through exactly the same topic as
every other command source here. Poses come off the solver and leave as ROS
topics; there is no UDP hop in between.

    ros2 launch rokoko_glove_control glove.launch.py side:=right

Start the solver first; it is what turns raw sensor poses into a skeleton:

    ~/.local/share/rokoko-device-sdk/bin/rkk-hand-solver

Units
-----
`motor_commands` carries the aligned units the rest of the workspace uses:
finger flexion axes as a closure fraction 0 (open) .. 1 (closed), wrist and
thumb-abduction axes in radians. Ticks are produced downstream by
`aligned_interface` (real hand) or `driver_interface` (simulation) from their
shared `calib.<axis>.tick_min/tick_max` parameters, so the glove path is
calibrated in the same one place as everything else.

The glove's own clamp ranges already coincide with the model's joint limits
(+/-90 deg rotation = +/-1.5708 rad, +/-45 deg = +/-0.7854 rad), so the wrist
conversion is a plain degrees-to-radians. Where `rh8d_mapping` would map a
channel onto 0..4095, this maps it onto the axis's unit range - the same
normalized position, expressed in the units this workspace speaks.

What a channel *means* comes from `rh8d_mapping` (its ranges, the thumb
gate); which model axis each one drives is `CHANNEL_FOR_AXIS` here, because
the model's joint names are anatomically accurate where the demo config's
labels were not. See the note on that table.

Safety
------
- Nothing is published until a pose actually arrives, so the hand is not
  commanded on startup.
- Commands are slew limited (`max_range_per_second`), which bounds the first
  move after the glove appears. Without it the hand snaps from wherever it is
  to wherever your hand happens to be.
- A stale stream keeps publishing the last pose, so the hand holds position
  instead of relaxing or jumping.
- Wrist rotation is measured from the forearm and tared at startup, so hold
  your forearm in a neutral pose as the node comes up.
"""
import math
import struct
import sys
from pathlib import Path

import numpy

# The glove modules sit at the package root and import each other flatly;
# both layouts (installed beside this script, or the source tree) work.
# Installed, the glove modules sit beside this script; in the source tree
# they are one level up in rokoko_glove_control/. They import each other
# flatly, so the directory holding them has to be on the path.
_HERE = Path(__file__).resolve().parent
for _candidate in (_HERE, _HERE.parent / 'rokoko_glove_control'):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import rh8d_mapping
import rgmp_client_rh8d
from hand_joints import (TWIST_AXIS, WRIST_SIGNS, Calibration, q_normalize,
                         twist_angle)
from rgmp_client_rh8d import DEFAULT_CALIBRATION, SOLVER_PORT, HandReader

#: Aligned-unit range of every motor axis, without the joint prefix. These are
#: the model's joint limits; test/test_glove_mapping.py checks them against
#: the URDF so this copy cannot drift from rh8d_macro.xacro.
AXIS_RANGES = {
    'wrist_rotation_joint': (-1.5708, 1.5708),
    'wrist_adduction_joint': (-0.7854, 0.7854),
    'wrist_flexion_joint': (-0.7854, 0.7854),
    'thumb_adduction_joint': (-0.6, 0.7854),
    'thumb_flexion_joint': (0.0, 1.0),
    'index_flexion_joint': (0.0, 1.0),
    'middle_flexion_joint': (0.0, 1.0),
    'ring_little_flexion_joint': (0.0, 1.0),
}

#: Which glove channel drives which model axis.
#:
#: Deliberately NOT rh8d_mapping.classify_joint. That function crosses the
#: wrist pair to compensate for the demo config's labels ("Wrist Adduction"
#: driven by the glove's flexion channel), and it matches on substrings, so
#: it cannot tell "Wrist Adduction" from "r_wrist_adduction_joint". The
#: model's joint names are anatomically accurate - measured off the URDF:
#:
#:   +r_wrist_flexion_joint   -> fingertips move palmar  = flexion
#:   +r_wrist_adduction_joint -> fingertips move ulnar   = adduction
#:   +r_thumb_adduction_joint -> thumb crosses the palm  = adduction
#:
#: and the glove channels are documented the same way, so the routing is a
#: direct name-to-name match. Sending the wrist through classify_joint would
#: cross a pair that is already straight.
CHANNEL_SIGN = {
    # Direction is a separate question from routing, and the glove does not
    # keep to its documented signs: both wrist axes come out reversed against
    # this model on the hardware. Rotation and the thumb read the right way
    # round. Anything absent here runs positive.
    'wrist_flexion': -1.0,
    'wrist_abduction': -1.0,
}

CHANNEL_FOR_AXIS = {
    'wrist_rotation_joint': 'wrist_rotation',
    'wrist_flexion_joint': 'wrist_flexion',
    'wrist_adduction_joint': 'wrist_abduction',
    'thumb_adduction_joint': 'thumb_abduction',
    'thumb_flexion_joint': 'thumb_flexion',
    'index_flexion_joint': 'index_flexion',
    'middle_flexion_joint': 'middle_flexion',
    'ring_little_flexion_joint': 'ring_flexion',
}


def channel_to_units(channel, value, axis_range):
    """A glove channel as the aligned unit of the axis it drives.

    The aligned-unit twin of `rh8d_mapping.channel_to_dynamixel`: identical
    normalization, mapped onto the axis range instead of onto 0..4095. Under
    the default tick calibration the two produce the same physical position.

    numpy.interp clamps at the ends of the input range, which is exactly the
    saturation each channel wants. It is called eight times per cycle at
    50 Hz, so its overhead is irrelevant here - unlike in hand_joints, where
    the same call rate would be thousands of times a second.
    """
    source = rh8d_mapping.WRIST_CHANNEL_RANGES.get(channel, (0.0, 1.0))
    return float(numpy.interp(value, source, axis_range))


class RollTracker:
    """A wrapping absolute angle turned into continuous travel from a neutral.

    The forearm roll arrives world-referenced, so its absolute value is
    meaningless on its own (it reads around -320 deg at rest) and it wraps at
    +/-180. This unwraps the wrapping and subtracts the first reading, so the
    channel is travel away from however the forearm was held at startup.
    """

    def __init__(self):
        self.previous = None
        self.continuous = 0.0
        self.neutral = 0.0

    def update(self, raw_deg):
        # math.remainder puts the step in [-180, 180], which is the shortest
        # way round - so a wrap reads as the small move it was.
        if self.previous is None:
            self.continuous = raw_deg
            self.neutral = raw_deg
        else:
            self.continuous += math.remainder(raw_deg - self.previous, 360.0)
        self.previous = raw_deg
        return self.continuous - self.neutral


class ForearmReader(HandReader):
    """HandReader that also tracks the forearm's roll about its own long axis.

    The solver streams the forearm as a world-referenced TRANSFORM and the
    hand as an orientation *relative to that forearm*, so the wrist rotation
    hand_joints derives is the wrist's own axial twist - anatomically almost
    nothing, because pronation and supination happen in the forearm, above
    the wrist. Rotating your arm therefore barely moves it. The forearm's own
    roll is in the same frame, unused; this picks it out.

    Kept here rather than in hand_joints so the glove modules stay as the
    demo tool has them.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._rolls = {}
        self._trackers = {}

    def _on_data(self, device_id, group_id, ts_us, payload, layout):
        super()._on_data(device_id, group_id, ts_us, payload, layout)
        model = self._models.get(device_id)
        if model is None or group_id != model.group_id:
            return
        # the one TRANSFORM slot is the forearm root: position then quaternion
        slot = next((s for s in model.slots.values() if s[1] == 7), None)
        if slot is None:
            return
        try:
            quaternion = struct.unpack_from('<4f', payload, (slot[0] + 3) * 4)
        except struct.error:
            return
        raw = twist_angle(q_normalize(quaternion), TWIST_AXIS)
        tracker = self._trackers.setdefault(device_id, RollTracker())
        # same sign convention hand_joints applies to its own rotation
        self._rolls[device_id] = (tracker.update(raw)
                                  * WRIST_SIGNS[1] * model.mirror)

    def forearm_roll(self, device_id):
        """Degrees of forearm roll since startup, or None before the first frame."""
        return self._rolls.get(device_id)


class RokokoGloveNode(Node):

    def __init__(self):
        super().__init__('rokoko_glove_node')
        self.glove_side = self.declare_parameter('side', 'right').value
        prefix = self.declare_parameter('joint_prefix', '').value
        self.jp = prefix or ('l_' if self.glove_side == 'left' else 'r_')
        host = self.declare_parameter('solver_host', '127.0.0.1').value
        port = self.declare_parameter('solver_port', SOLVER_PORT).value
        self.rate = float(self.declare_parameter('rate', 50.0).value)
        calibration_file = self.declare_parameter('calibration_file', '').value
        self.wrist_scale = float(self.declare_parameter('wrist_scale', 1.0).value)
        self.lock_wrist = bool(self.declare_parameter('lock_wrist', False).value)
        # A pose older than this still gets published - the hand holds rather
        # than jumping - but it is called out in the log.
        self.stale_after = float(self.declare_parameter('stale_timeout', 0.5).value)
        # Bounds how fast a command may cross an axis's range. The first move
        # after the glove appears is the one this exists for. 0 disables it.
        self.max_rate = float(
            self.declare_parameter('max_range_per_second', 3.0).value)

        # Joint name -> (channel, aligned range), in a fixed order. A joint no
        # channel drives is left out of the message entirely rather than
        # commanded to a midpoint, so nothing else fights over it.
        self.targets = {}
        unmapped = []
        for axis, axis_range in AXIS_RANGES.items():
            joint = self.jp + axis
            channel = CHANNEL_FOR_AXIS.get(axis)
            if channel is None:
                unmapped.append(joint)
                continue
            self.targets[joint] = (channel, axis_range)
        if unmapped:
            self.get_logger().warn(
                f'no glove channel drives {unmapped}; left uncommanded')

        calibration = self._load_calibration(calibration_file)
        self.reader = ForearmReader(host, port, calibration)
        self.reader.__enter__()

        self.publisher = self.create_publisher(JointState, 'motor_commands', 10)
        self.commanded = {}
        self.last_frame = None
        self.last_change = None
        self.was_stale = False
        self.waiting_logged = False
        self.last_tick = None

        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'rokoko glove -> motor_commands: {self.glove_side} glove on '
            f'{host}:{port}, joint_prefix="{self.jp}", {self.rate:.0f} Hz, '
            f'{len(self.targets)} axes driven')

    def _default_calibration_paths(self):
        """Where --calibrate leaves its file, installed or in the source tree."""
        # DEFAULT_CALIBRATION sits beside rgmp_client_rh8d; under
        # --symlink-install that module is a link, so resolve it to reach the
        # source-tree copy the tools actually write.
        beside_module = Path(rgmp_client_rh8d.__file__).resolve().with_name(
            Path(DEFAULT_CALIBRATION).name)
        return [Path(DEFAULT_CALIBRATION), beside_module]

    def _load_calibration(self, path):
        """The glove's own open/fist calibration - not the hand's tick calibration."""
        if not path:
            for candidate in self._default_calibration_paths():
                if candidate.is_file():
                    path = str(candidate)
                    break
        if not path:
            self.get_logger().warn(
                'no glove calibration found: closure uses default ranges, so '
                'the fingers will not reach their ends of travel, and the '
                'thumb flexion channel tends to track adduction instead. '
                'Produce one with '
                '"ros2 run rokoko_glove_control glove_calibrate".')
            return Calibration()
        import json
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f'cannot read glove calibration {path} ({exc}); using defaults')
            return Calibration()
        self.get_logger().info(f'using glove calibration {path}')
        return Calibration.from_dict(data)

    # ── pose selection ──────────────────────────────────────────────────────

    def select_pose(self):
        poses = self.reader.latest()
        if not poses:
            return None
        if self.glove_side == 'auto':
            return poses[0]
        for pose in poses:
            if pose.side == self.glove_side:
                return pose
        return None

    # ── periodic ────────────────────────────────────────────────────────────

    def tick(self):
        now = self.get_clock().now()
        elapsed = 1.0 / self.rate
        if self.last_tick is not None:
            elapsed = max((now - self.last_tick).nanoseconds * 1e-9, 1e-4)
        self.last_tick = now

        pose = self.select_pose()
        if pose is None:
            if not self.waiting_logged:
                self.waiting_logged = True
                self.get_logger().info(
                    f'waiting for a {self.glove_side} hand from the solver '
                    '(is rkk-hand-solver running, glove connected?)')
            return          # publish nothing: never command the hand blind
        self.waiting_logged = False

        if pose.timestamp_us != self.last_frame:
            self.last_frame = pose.timestamp_us
            self.last_change = now
        stale = (self.last_change is not None
                 and (now - self.last_change).nanoseconds * 1e-9 > self.stale_after)
        if stale != self.was_stale:
            self.was_stale = stale
            if stale:
                self.get_logger().warn(
                    'glove stream stale; holding the last pose')
            else:
                self.get_logger().info('glove stream live again')

        self.publish(self.glove_channels(pose), elapsed)

    def glove_channels(self, pose):
        """The eight glove channels for a pose, keyed by channel name.

        Finger closure and thumb adduction arrive already normalized 0..1 by
        the glove calibration in hand_joints, the wrist angles in degrees.
        This picks them out, applies the axis directions, and pairs the ring
        and little fingers. Ranges and the thumb gate come from rh8d_mapping,
        the transport-neutral half of the glove code; channel_to_units does
        the clamping.
        """
        def signed(channel, value):
            return CHANNEL_SIGN.get(channel, 1.0) * value

        closure = {finger.name: finger.closure for finger in pose.fingers}
        glove = {
            'thumb_abduction': signed('thumb_abduction', pose.thumb.adduction),
            'thumb_flexion': closure.get('thumb', 0.0),
            'index_flexion': closure.get('index', 0.0),
            'middle_flexion': closure.get('middle', 0.0),
            # one motor drives both, so average them: a single twitchy finger
            # should not swing the pair
            'ring_flexion': 0.5 * (closure.get('ring', 0.0)
                                   + closure.get('little', 0.0)),
            'wrist_rotation': 0.0,
            'wrist_abduction': 0.0,
            'wrist_flexion': 0.0,
        }
        glove['thumb_flexion'] = rh8d_mapping.apply_thumb_gate(
            glove['thumb_abduction'], glove['thumb_flexion'])

        if not self.lock_wrist:
            # Rotation comes from the forearm, not the wrist joint - see
            # ForearmReader. Until the first forearm frame arrives, fall back
            # to the wrist twist rather than freezing at zero.
            roll = self.reader.forearm_roll(pose.device_id)
            if roll is None:
                roll = pose.wrist.rotation_deg
            for channel, value in (
                    ('wrist_rotation', roll),
                    ('wrist_abduction', pose.wrist.adduction_deg),
                    ('wrist_flexion', pose.wrist.flexion_deg)):
                glove[channel] = signed(channel, value) * self.wrist_scale
        return glove

    def publish(self, glove, elapsed):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for joint, (channel, axis_range) in self.targets.items():
            target = channel_to_units(channel, glove.get(channel, 0.0), axis_range)
            message.name.append(joint)
            message.position.append(self.slew(joint, target, axis_range, elapsed))
        self.publisher.publish(message)

    def slew(self, joint, target, axis_range, elapsed):
        """Limit how far a command may move in one cycle."""
        previous = self.commanded.get(joint)
        if previous is None or self.max_rate <= 0.0:
            self.commanded[joint] = target
            return target
        span = axis_range[1] - axis_range[0]
        step = self.max_rate * span * elapsed
        value = previous + max(-step, min(step, target - previous))
        self.commanded[joint] = value
        return value

    def destroy_node(self):
        try:
            self.reader.__exit__(None, None, None)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RokokoGloveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print(f'cannot reach the Rokoko solver: {exc}\n'
              f'Is rkk-hand-solver running? It serves solved hands on '
              f'127.0.0.1:{SOLVER_PORT}.', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())
