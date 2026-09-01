#!/usr/bin/env python3
"""The glove -> motor_commands path, without a glove or a solver.

Poses are synthesised, so every channel can be driven to a known value and
the published command checked exactly. Covers the parts that go wrong
silently: which channel drives which joint (including the crossed wrist pair
and the thumb's two spellings), the unit conversion at both ends of travel,
and that this package's copy of the axis ranges still matches the URDF.
"""
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent
for _candidate in (_PKG / 'rokoko_glove_control', _PKG / 'scripts'):
    sys.path.insert(0, str(_candidate))

os.environ.setdefault(
    'ROS_DOMAIN_ID',
    str(int(os.environ.get('RH8D_TEST_DOMAIN_BASE', '88')) + 9))

import rclpy
from sensor_msgs.msg import JointState

import rh8d_mapping
from hand_joints import FingerState, HandPose, ThumbState, WristState

FAILS = []


def check(ok, message):
    print(('  PASS  ' if ok else '  FAIL  ') + message, flush=True)
    if not ok:
        FAILS.append(message)
    return bool(ok)


def section(title):
    print(f'\n--- {title} ---', flush=True)


def make_pose(closures=None, thumb_abduction=0.0, flexion_deg=0.0,
              adduction_deg=0.0, rotation_deg=0.0, side='right', ts=1):
    """A HandPose holding exactly the values a test wants to see come out."""
    closures = closures or {}
    fingers = [
        FingerState(name=name, joint_angles=[0.0, 0.0, 0.0], curl_deg=0.0,
                    closure=closures.get(name, 0.0), spread_deg=0.0)
        for name in ('thumb', 'index', 'middle', 'ring', 'little')
    ]
    return HandPose(
        device_id=1, side=side, timestamp_us=ts,
        wrist=WristState(flexion_deg=flexion_deg, adduction_deg=adduction_deg,
                         rotation_deg=rotation_deg),
        thumb=ThumbState(adduction_deg=0.0, adduction=thumb_abduction,
                         palmar_abduction_deg=0.0, span_deg=0.0),
        fingers=fingers)


class FakeReader:
    """Stands in for ForearmReader: no socket, no solver, no glove."""

    def __init__(self, *args, **kwargs):
        self.poses = []
        self.roll = None          # None = no forearm frame seen yet

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def latest(self):
        return list(self.poses)

    def forearm_roll(self, device_id):
        return self.roll


# ── channel routing ─────────────────────────────────────────────────────────

def test_routing():
    section('which channel drives which model axis')
    import glove_node
    expected = {
        'wrist_rotation_joint': 'wrist_rotation',
        # straight through: the model's joint names are anatomically accurate,
        # so flexion drives flexion and adduction drives abduction (ulnar)
        'wrist_flexion_joint': 'wrist_flexion',
        'wrist_adduction_joint': 'wrist_abduction',
        'thumb_adduction_joint': 'thumb_abduction',
        'thumb_flexion_joint': 'thumb_flexion',
        'index_flexion_joint': 'index_flexion',
        'middle_flexion_joint': 'middle_flexion',
        'ring_little_flexion_joint': 'ring_flexion',
    }
    check(glove_node.CHANNEL_FOR_AXIS == expected,
          'every model axis routes to its channel, wrist pair uncrossed')
    check(set(glove_node.CHANNEL_FOR_AXIS) == set(glove_node.AXIS_RANGES),
          'the routing table covers exactly the axes that have a range')

    # The wrist pair is a documented divergence from the shared mapping file.
    # Everything else must still agree with it, so a change there is caught.
    section('agreement with rh8d_mapping.classify_joint')
    wrist = {'wrist_flexion_joint', 'wrist_adduction_joint'}
    for prefix in ('r_', 'l_'):
        wrong = {axis: (rh8d_mapping.classify_joint(prefix + axis), channel)
                 for axis, channel in expected.items() if axis not in wrist
                 and rh8d_mapping.classify_joint(prefix + axis) != channel}
        check(not wrong,
              f'{prefix}: the non-wrist axes agree with the shared mapping'
              + (f' - differ: {wrong}' if wrong else ''))
    crossed = {axis: rh8d_mapping.classify_joint('r_' + axis) for axis in sorted(wrist)}
    check(crossed == {'wrist_flexion_joint': 'wrist_abduction',
                      'wrist_adduction_joint': 'wrist_flexion'},
          f'classify_joint still crosses the wrist for the demo config '
          f'({crossed}); this node deliberately does not')
    check(rh8d_mapping.classify_joint('Thumb Adduction') == 'thumb_abduction',
          "the demo config's 'Thumb Adduction' spelling still routes")
    check(rh8d_mapping.classify_joint('r_palm_ir_joint') is None,
          'a joint no channel drives returns None')


def test_roll_tracker():
    section('forearm roll: unwrapping and the startup tare')
    import glove_node
    t = glove_node.RollTracker()
    check(t.update(-320.0) == 0.0,
          'the first reading is the neutral, whatever its absolute value')
    check(abs(t.update(-290.0) - 30.0) < 1e-9,
          'travel is measured from that neutral')
    check(abs(t.update(-320.0)) < 1e-9, 'and returns to zero')

    # a full turn crossing the +/-180 wrap must stay continuous
    t = glove_node.RollTracker()
    t.update(170.0)
    check(abs(t.update(-175.0) - 15.0) < 1e-9,
          'crossing +180 -> -180 reads as continuing, not as a 345 deg jump')
    check(abs(t.update(-170.0) - 20.0) < 1e-9, 'and keeps counting past it')
    t2 = glove_node.RollTracker()
    t2.update(-170.0)
    check(abs(t2.update(175.0) + 15.0) < 1e-9, 'the same the other way round')

    # 180 degrees of forearm travel must be reachable, where the wrist twist
    # this replaces only ever managed about 45
    t = glove_node.RollTracker()
    t.update(0.0)
    check(abs(t.update(-180.0) + 180.0) < 1e-6 or abs(t.update(179.0) - 179.0) < 1e-6,
          'a half turn is representable')


def test_default_inversions():
    section('axis directions')
    import glove_node
    # Measured on the hardware, not derivable from the model: both wrist axes
    # come out reversed, rotation and the thumb do not.
    check(glove_node.CHANNEL_SIGN == {'wrist_flexion': -1.0,
                                      'wrist_abduction': -1.0},
          f'the wrist axes are inverted, the rest run positive '
          f'({glove_node.CHANNEL_SIGN})')


def test_no_transport_dependency():
    section('the node carries no transport baggage')
    source = (_PKG / 'scripts' / 'glove_node.py').read_text()
    imports = [line.strip() for line in source.splitlines()
               if line.startswith(('import ', 'from '))
               and ('udp' in line.lower() or 'socket' in line)]
    check(not imports,
          'glove_node imports nothing UDP- or socket-related'
          + (f' - found: {imports}' if imports else ''))
    check(not (_PKG / 'rokoko_glove_udp.py').exists()
          and not (_PKG / 'rokoko_glove_control' / 'rokoko_glove_udp.py').exists(),
          'and the UDP publisher is gone from the package')


# ── the axis table against the URDF ─────────────────────────────────────────

def test_axis_ranges_match_urdf():
    section('axis ranges against the URDF')
    import glove_node
    xacro_path = _PKG.parent / 'seed_rh8d_description' / 'urdf' / 'rh8d.urdf.xacro'
    try:
        urdf = subprocess.run(
            ['xacro', str(xacro_path), 'side:=right', 'use_ros2_control:=false'],
            capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.TimeoutExpired) as exc:
        check(False, f'could not expand the URDF: {exc}')
        return
    if urdf.returncode != 0:
        check(False, f'xacro failed: {urdf.stderr.strip()[:200]}')
        return

    limits = {}
    for joint in ET.fromstring(urdf.stdout).findall('joint'):
        limit = joint.find('limit')
        if limit is not None and limit.get('lower') is not None:
            limits[joint.get('name')] = (float(limit.get('lower')),
                                         float(limit.get('upper')))
    drifted = {}
    for axis, (low, high) in glove_node.AXIS_RANGES.items():
        if (low, high) == (0.0, 1.0):
            continue                    # a closure fraction, not a URDF limit
        actual = limits.get('r_' + axis)
        if (actual is None or abs(actual[0] - low) > 1e-3
                or abs(actual[1] - high) > 1e-3):
            drifted[axis] = {'urdf': actual, 'glove_node': (low, high)}
    check(not drifted,
          'the radian axis ranges match the URDF joint limits'
          + (f' - drifted: {drifted}' if drifted else ''))


# ── unit conversion ─────────────────────────────────────────────────────────

def test_conversion():
    section('channel value -> aligned unit')
    import glove_node
    convert = glove_node.channel_to_units

    check(convert('index_flexion', 0.0, (0.0, 1.0)) == 0.0
          and convert('index_flexion', 1.0, (0.0, 1.0)) == 1.0,
          'a finger channel is the closure fraction unchanged')
    check(convert('index_flexion', 2.5, (0.0, 1.0)) == 1.0
          and convert('index_flexion', -3.0, (0.0, 1.0)) == 0.0,
          'and is clamped to 0..1')

    rotation = (-1.5708, 1.5708)
    check(abs(convert('wrist_rotation', 90.0, rotation) - 1.5708) < 1e-4
          and abs(convert('wrist_rotation', -90.0, rotation) + 1.5708) < 1e-4,
          'wrist rotation +/-90 deg reaches +/-1.5708 rad')
    check(abs(convert('wrist_rotation', 45.0, rotation) - math.radians(45)) < 1e-3,
          'and is a plain degrees-to-radians in between')
    check(abs(convert('wrist_rotation', 180.0, rotation) - 1.5708) < 1e-4,
          'an angle past the range clamps at the limit')

    thumb = (-0.6, 0.7854)
    check(abs(convert('thumb_abduction', 0.0, thumb) + 0.6) < 1e-6,
          'thumb abduction 0 (spread wide) maps to the axis minimum')
    check(abs(convert('thumb_abduction', 1.0, thumb) - 0.7854) < 1e-6,
          'thumb abduction 1 (onto the palm) maps to the axis maximum')

    section('agreement with the tick path')
    # channel_to_units and rh8d_mapping.channel_to_dynamixel must place a
    # channel at the same fraction of travel; under the default 0..4095
    # calibration they are then the same physical position.
    worst = 0.0
    for channel, axis_range, values in (
            ('index_flexion', (0.0, 1.0), [0.0, 0.25, 0.5, 0.75, 1.0]),
            ('thumb_abduction', (-0.6, 0.7854), [0.0, 0.3, 1.0]),
            ('wrist_rotation', (-1.5708, 1.5708), [-90.0, -30.0, 0.0, 60.0, 90.0]),
            ('wrist_flexion', (-0.7854, 0.7854), [-45.0, 0.0, 45.0])):
        low, high = axis_range
        for value in values:
            units = convert(channel, value, axis_range)
            unit_fraction = (units - low) / (high - low)
            tick_fraction = rh8d_mapping.channel_to_dynamixel(channel, value) / 4095.0
            worst = max(worst, abs(unit_fraction - tick_fraction))
    check(worst < 1e-3,
          f'both paths place every channel at the same fraction of travel '
          f'(worst {worst:.2e})')


# ── the node ────────────────────────────────────────────────────────────────

def collect(node, seconds):
    received = []
    probe = rclpy.create_node('glove_probe')
    probe.create_subscription(JointState, 'motor_commands', received.append, 10)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        rclpy.spin_once(probe, timeout_sec=0.01)
    probe.destroy_node()
    return received


def last_values(node, seconds=2.5):
    messages = collect(node, seconds)
    if not messages:
        return None
    return dict(zip(messages[-1].name, messages[-1].position))


def test_node():
    section('the node, driven by synthetic poses')
    import glove_node
    glove_node.ForearmReader = FakeReader          # no solver, no glove
    node = glove_node.RokokoGloveNode()
    try:
        check(not collect(node, 1.5),
              'nothing is published before a pose arrives')

        node.reader.poses = [make_pose(
            closures={'thumb': 1.0, 'index': 1.0, 'middle': 1.0,
                      'ring': 1.0, 'little': 1.0},
            thumb_abduction=1.0, ts=2)]
        values = last_values(node)
        if not check(values is not None, 'a pose produces motor_commands'):
            return
        check(len(values) == 8, f'all 8 motor axes are commanded ({len(values)})')
        check(sorted(values) == sorted('r_' + a for a in glove_node.AXIS_RANGES),
              "with the model's joint names")
        closed = {n: v for n, v in values.items()
                  if n.endswith('_flexion_joint') and 'wrist' not in n}
        check(all(v > 0.99 for v in closed.values()),
              'a fist closes every flexion axis to ~1.0 ('
              + ', '.join(f'{n.replace("r_", "")}={v:.2f}'
                          for n, v in closed.items()) + ')')
        check(abs(values['r_thumb_adduction_joint'] - 0.7854) < 1e-3,
              f'thumb abduction 1.0 -> '
              f'{values["r_thumb_adduction_joint"]:.4f} rad')

        node.reader.poses = [make_pose(ts=3)]
        values = last_values(node)
        check(all(abs(values['r_' + a]) < 1e-3
                  for a in ('index_flexion_joint', 'middle_flexion_joint',
                            'ring_little_flexion_joint', 'thumb_flexion_joint')),
              'an open hand returns the flexion axes to ~0')

        section('the wrist pair, end to end')
        # driven one at a time, so a swap between the two cannot hide
        # both wrist axes are inverted by default, so +45 deg of glove
        # travel lands at the joint's negative limit
        node.reader.poses = [make_pose(flexion_deg=45.0, ts=4)]
        values = last_values(node)
        check(abs(values['r_wrist_flexion_joint'] + 0.7854) < 1e-3,
              f'glove flexion +45 deg reaches r_wrist_flexion_joint, inverted '
              f'({values["r_wrist_flexion_joint"]:+.4f} rad)')
        check(abs(values['r_wrist_adduction_joint']) < 1e-3,
              f'and leaves r_wrist_adduction_joint centred '
              f'({values["r_wrist_adduction_joint"]:+.4f} rad)')

        node.reader.poses = [make_pose(adduction_deg=45.0, ts=5)]
        values = last_values(node)
        check(abs(values['r_wrist_adduction_joint'] + 0.7854) < 1e-3,
              f'glove ulnar +45 deg reaches r_wrist_adduction_joint, inverted '
              f'({values["r_wrist_adduction_joint"]:+.4f} rad)')
        check(abs(values['r_wrist_flexion_joint']) < 1e-3,
              f'and leaves r_wrist_flexion_joint centred '
              f'({values["r_wrist_flexion_joint"]:+.4f} rad)')

        node.reader.poses = [make_pose(flexion_deg=-45.0, ts=6)]
        values = last_values(node)
        check(values['r_wrist_flexion_joint'] > 0.78,
              f'and the two ends of glove travel stay opposite '
              f'({values["r_wrist_flexion_joint"]:+.4f} rad)')

        node.reader.poses = [make_pose(rotation_deg=-90.0, ts=8)]
        values = last_values(node)
        check(abs(values['r_wrist_rotation_joint'] + 1.5708) < 1e-3,
              f'with no forearm frame yet, rotation falls back to the wrist '
              f'twist ({values["r_wrist_rotation_joint"]:+.4f} rad)')

        # once the forearm is being tracked it takes over: the wrist twist
        # barely moves, the forearm carries the pronation
        node.reader.roll = 90.0
        node.reader.poses = [make_pose(rotation_deg=-90.0, ts=9)]
        values = last_values(node)
        check(abs(values['r_wrist_rotation_joint'] - 1.5708) < 1e-3,
              f'forearm roll overrides the wrist twist '
              f'({values["r_wrist_rotation_joint"]:+.4f} rad from +90 deg of roll)')
        node.reader.roll = 45.0
        node.reader.poses = [make_pose(ts=10)]
        values = last_values(node)
        check(abs(values['r_wrist_rotation_joint'] - math.radians(45)) < 2e-3,
              f'and tracks it 1:1 in between '
              f'({values["r_wrist_rotation_joint"]:+.4f} rad from +45 deg)')
        node.reader.roll = 200.0
        node.reader.poses = [make_pose(ts=11)]
        values = last_values(node)
        check(abs(values['r_wrist_rotation_joint'] - 1.5708) < 1e-3,
              'and clamps past the joint limit rather than wrapping')
        node.reader.roll = None

        section('the wrong glove hand')
        check(last_values(node, 0.8) is not None, 'publishing while a right hand is up')
        node.reader.poses = [make_pose(closures={'index': 1.0}, side='left', ts=7)]
        check(not collect(node, 2.0),
              'a left-hand pose is ignored by a node following the right glove: '
              'it stops publishing rather than commanding, so the hand holds '
              'its last target')
    finally:
        node.destroy_node()


def test_lock_wrist():
    section('lock_wrist:=true')
    import glove_node
    glove_node.ForearmReader = FakeReader
    node = glove_node.RokokoGloveNode()
    try:
        node.lock_wrist = True
        node.reader.poses = [make_pose(flexion_deg=45.0, rotation_deg=80.0,
                                       closures={'index': 1.0}, ts=8)]
        values = last_values(node, 3.0)
        wrist = {n: v for n, v in values.items() if 'wrist' in n}
        check(all(abs(v) < 1e-3 for v in wrist.values()),
              'the wrist axes stay centred ('
              + ', '.join(f'{n.replace("r_", "")}={v:+.3f}'
                          for n, v in wrist.items()) + ')')
        check(values['r_index_flexion_joint'] > 0.99,
              'while the fingers still follow the glove')
    finally:
        node.destroy_node()


def test_slew():
    section('slew limiting')
    import glove_node
    glove_node.ForearmReader = FakeReader
    node = glove_node.RokokoGloveNode()
    try:
        node.max_rate = 1.0                    # one full range per second
        node.commanded['r_index_flexion_joint'] = 0.0
        value = node.slew('r_index_flexion_joint', 1.0, (0.0, 1.0), 0.1)
        check(abs(value - 0.1) < 1e-9,
              f'a step is capped at max_range_per_second ({value:.3f} in 0.1 s)')
        for _ in range(20):
            value = node.slew('r_index_flexion_joint', 1.0, (0.0, 1.0), 0.1)
        check(abs(value - 1.0) < 1e-9, f'and converges on the target ({value:.3f})')
        node.max_rate = 0.0
        node.commanded['r_index_flexion_joint'] = 0.0
        check(node.slew('r_index_flexion_joint', 1.0, (0.0, 1.0), 0.1) == 1.0,
              'max_range_per_second 0 disables the limit')
    finally:
        node.destroy_node()


def main():
    print('\n=== Rokoko glove -> motor_commands ===', flush=True)
    test_routing()
    test_axis_ranges_match_urdf()
    test_conversion()
    rclpy.init()
    try:
        test_node()
        test_lock_wrist()
        test_slew()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print('=' * 70, flush=True)
    if FAILS:
        print(f'{len(FAILS)} FAILURE(S):', flush=True)
        for f in FAILS:
            print('  - ' + f, flush=True)
        return 1
    print('ALL CHECKS PASSED', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
