#!/usr/bin/env python3
"""Sim-aligned interface for the real hand.

Bridges the driver's native tick protocol to the same topics, joint names,
and units the simulation uses, so code written against the sim runs on the
real hand unchanged. Purely additive: the native R_/L_ topics keep working.

Units (matching rh8d_coupling_node with normalized_fingers:=true):
  flexion axes    closure fraction, 0.0 open .. 1.0 closed
  wrist axes and thumb adduction    radians

Interface
  in:  motor_commands (sensor_msgs/JointState) - motor-axis names
       (<jp>{thumb,index,middle}_flexion_joint, <jp>ring_little_flexion_joint,
       <jp>wrist_{rotation,adduction,flexion}_joint, <jp>thumb_adduction_joint;
       jp = joint_prefix). Optional velocity entries are converted with
       speed_scale (ignored when speed_scale is 0).
  in:  hand_controller/joint_trajectory (trajectory_msgs/JointTrajectory) -
       the sim controller's topic. Accepts motor-axis names and/or phalanx
       joint names (mimic leaders or the full independent-mode set); a
       finger's closure is estimated as sum(position)/sum(upper limit) over
       the chain joints present. The last trajectory point is executed.
  out: motor_states (sensor_msgs/JointState) - present motor positions in
       the units above, converted from the driver's state topic.

Calibration: per-motor tick endpoints are ROS parameters
(calib.<axis_without_prefix>.{tick_min,tick_max}). tick_min corresponds to
the axis minimum (fingers: open; wrist: lower rad limit), tick_max to the
maximum. Swap the two values to invert a motor's direction. THE DEFAULTS
(0/4095 over the full model range) ARE NOT HARDWARE-CALIBRATED - verify on
the real hand and override in the driver YAML.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from seed_hand_msgs.msg import AllJoints, JointListSetSpeedPos, JointSetSpeedPos

# model ranges of the aligned units (keep in sync with rh8d_coupling_node)
PASSTHROUGH = {  # axis -> (rad_min, rad_max)
    'wrist_rotation_joint': (-1.5708, 1.5708),
    'wrist_adduction_joint': (-0.7854, 0.7854),
    'wrist_flexion_joint': (-0.7854, 0.7854),
    'thumb_adduction_joint': (-0.6, 0.7854),
}
# flexion axis -> phalanx chains it drives: (joint suffix, upper limit)
_finger = lambda f: [(f'{f}_proximal_joint', 1.57),
                     (f'{f}_medial_joint', 1.57),
                     (f'{f}_distal_joint', 1.0)]
CHAINS = {
    'thumb_flexion_joint': [[('thumb_proximal_joint', 0.8),
                             ('thumb_medial_joint', 1.57),
                             ('thumb_distal_joint', 0.4)]],
    'index_flexion_joint': [_finger('index')],
    'middle_flexion_joint': [_finger('middle')],
    'ring_little_flexion_joint': [_finger('ring'), _finger('little')],
}


def clamp(v, lo, hi):
    return min(max(v, lo), hi)


class AlignedInterface(Node):

    def __init__(self):
        super().__init__('hand_aligned_interface')
        tp = self.declare_parameter('topic_prefix', 'R_').value
        self.jp = self.declare_parameter('joint_prefix', 'r_').value
        self.default_speed = self.declare_parameter('default_speed', -1).value
        # device speed units (0..1023) per aligned-unit/s; 0 = ignore velocities
        self.speed_scale = self.declare_parameter('speed_scale', 0.0).value

        # axis (with prefix) -> (unit_min, unit_max, tick_min, tick_max)
        self.calib = {}
        for axis, (lo, hi) in PASSTHROUGH.items():
            self._add_axis(axis, lo, hi)
        for axis in CHAINS:
            self._add_axis(axis, 0.0, 1.0)

        # phalanx joint (with prefix) -> (flexion axis, upper limit)
        self.chain_joints = {}
        for axis, chains in CHAINS.items():
            for chain in chains:
                for suffix, upper in chain:
                    self.chain_joints[self.jp + suffix] = (self.jp + axis, upper)

        self.pub_cmd = self.create_publisher(
            JointListSetSpeedPos, tp + 'speed_position', 10)
        self.pub_state = self.create_publisher(JointState, 'motor_states', 10)
        self.create_subscription(JointState, 'motor_commands', self.on_motor_cmd, 10)
        self.create_subscription(JointTrajectory, 'hand_controller/joint_trajectory',
                                 self.on_trajectory, 10)
        self.create_subscription(AllJoints, tp + 'Joints', self.on_joints, 10)
        self.get_logger().info(
            f'aligned interface up: {len(self.calib)} motor axes, '
            f'topic_prefix="{tp}", joint_prefix="{self.jp}" '
            '(tick calibration defaults are NOT hardware-verified)')

    def _add_axis(self, axis, unit_min, unit_max):
        t0 = self.declare_parameter(f'calib.{axis}.tick_min', 0).value
        t1 = self.declare_parameter(f'calib.{axis}.tick_max', 4095).value
        self.calib[self.jp + axis] = (unit_min, unit_max, t0, t1)

    # ── unit conversion ──────────────────────────────────────────────────────

    def to_ticks(self, name, value):
        lo, hi, t0, t1 = self.calib[name]
        f = (clamp(value, lo, hi) - lo) / (hi - lo)
        return int(round(t0 + f * (t1 - t0)))

    def to_units(self, name, ticks):
        lo, hi, t0, t1 = self.calib[name]
        if t0 == t1:
            return lo
        return lo + (ticks - t0) / (t1 - t0) * (hi - lo)

    def _speed(self, velocity):
        if velocity and self.speed_scale > 0:
            return clamp(int(abs(velocity) * self.speed_scale), 1, 1023)
        return self.default_speed

    def _send(self, targets):
        """targets: {motor axis name: (unit value, device speed)}"""
        if not targets:
            return
        msg = JointListSetSpeedPos()
        for name, (value, speed) in targets.items():
            j = JointSetSpeedPos()
            j.name = name
            j.target_pos = self.to_ticks(name, value)
            j.target_speed = speed
            msg.joints.append(j)
        self.pub_cmd.publish(msg)

    # ── callbacks ────────────────────────────────────────────────────────────

    def on_motor_cmd(self, msg):
        targets = {}
        for i, name in enumerate(msg.name):
            if name not in self.calib:
                continue
            vel = msg.velocity[i] if i < len(msg.velocity) else None
            targets[name] = (msg.position[i], self._speed(vel))
        self._send(targets)

    def on_trajectory(self, msg):
        if not msg.points:
            return
        point = msg.points[-1]
        targets = {}
        closure = {}  # flexion axis -> [sum positions, sum uppers]
        for name, pos in zip(msg.joint_names, point.positions):
            if name in self.calib:  # motor axis commanded directly
                targets[name] = (pos, self.default_speed)
            elif name in self.chain_joints:  # phalanx joint -> closure estimate
                axis, upper = self.chain_joints[name]
                s = closure.setdefault(axis, [0.0, 0.0])
                s[0] += max(0.0, pos)
                s[1] += upper
        for axis, (pos_sum, upper_sum) in closure.items():
            if axis not in targets:
                targets[axis] = (pos_sum / upper_sum, self.default_speed)
        self._send(targets)

    def on_joints(self, msg):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for j in msg.joints:
            if j.name not in self.calib:
                continue
            out.name.append(j.name)
            out.position.append(self.to_units(j.name, j.present_position))
            if self.speed_scale > 0:
                out.velocity.append(j.present_speed / self.speed_scale)
        self.pub_state.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(AlignedInterface())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
