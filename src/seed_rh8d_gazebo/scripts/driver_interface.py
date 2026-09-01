#!/usr/bin/env python3
"""Driver-native interface for the simulated hand.

The mirror of seed_hand_driver's aligned_interface.py: that one puts the
sim's topics and units on the real hand, this one puts the driver's native
tick interface on the simulation, so code written against the hardware (the
user samples, existing scripts) runs against Gazebo unchanged.

  publish    <tp>Joints              seed_hand_msgs/AllJoints
  publish    <tp>Main_Boards         seed_hand_msgs/AllMainBoards
  subscribe  <tp>speed_position      seed_hand_msgs/JointListSetSpeedPos
  subscribe  <tp>stiffness           seed_hand_msgs/JointListSetStiffness
  subscribe  <tp>clear_error         seed_hand_msgs/ClearHWError
  subscribe  <tp>shutdown_condition  seed_hand_msgs/SetShutdownCond

Motor names and bus IDs match seed_hand_driver/config/RH8D_<L|R>.yaml (IDs
run base_id .. base_id+8, main board first). Units are the driver's: position
0..4095 ticks, speed 0..1023 with 0 = maximum and -1 = keep the previous
speed. Joints are addressable by name or by bus ID as a numeric string, as on
the hardware.

Calibration uses the same parameters and semantics as aligned_interface:
calib.<axis_without_prefix>.{tick_min,tick_max}, where tick_min is the axis
minimum (fingers: open; wrist: lower rad limit) and tick_max the maximum.
Swap the two to invert a motor. One calibration block therefore serves both
interfaces. THE DEFAULTS (0/4095 over the full model range) ARE NOT
HARDWARE-CALIBRATED - verify on the real hand and override in both configs.

Output to the simulation (parameter 'output')
  motor_commands  JointState in aligned units (fingers 0..1 closure, wrist
                  radians) on motor_commands, for finger_coupling:=independent
                  - feeds rh8d_coupling_node with normalized_fingers:=true.
  trajectory      JointTrajectory on hand_controller/joint_trajectory, for
                  finger_coupling:=mimic - the leader joint of each finger.

target_speed is honoured by rate-limiting the command internally, the way the
servo's profile velocity does; a single trajectory point cannot carry a
per-joint speed.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from seed_hand_msgs.msg import (
    AllJoints, AllMainBoards, LoneJoint, LoneMainBoard,
    JointListSetSpeedPos, JointListSetStiffness,
    ClearHWError, SetShutdownCond,
)

# model ranges of the aligned units (keep in sync with rh8d_coupling_node
# and seed_hand_driver/scripts/aligned_interface.py)
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
# bus IDs, as laid out in the driver YAMLs: main board then the 8 motors
BUS_ORDER = ['main_board'] + list(PASSTHROUGH) + list(CHAINS)

RAW_POS_MAX = 4095
RAW_SPEED_MAX = 1023
DEFAULT_STIFFNESS = 8
SECURITY_PERIOD = 30.0   # s between two clear_error / stiffness writes


def clamp(v, lo, hi):
    return min(max(v, lo), hi)


class _Motor:
    """One of the 8 actuators, tracked in its aligned unit."""

    def __init__(self, name, key, bus_id, unit_min, unit_max, ticks, direct):
        self.name = name
        self.key = key                 # axis name without the joint prefix
        self.bus_id = bus_id
        self.unit_min = unit_min
        self.unit_max = unit_max
        self.tick_min, self.tick_max = ticks
        self.direct = direct           # wrist/abduction axis, or a tendon
        self.target = None             # commanded value, None = never written
        self.cmd = None                # rate-limited value actually sent
        self.rate = 0.0                # units/s, 0 = as fast as the sim allows
        self.target_speed_raw = 0
        self.stiffness = DEFAULT_STIFFNESS
        self.time_last_clear = None
        self.time_last_stiffness = None

    def to_ticks(self, value):
        f = (clamp(value, self.unit_min, self.unit_max) - self.unit_min) \
            / (self.unit_max - self.unit_min)
        return int(round(clamp(self.tick_min + f * (self.tick_max - self.tick_min),
                               0, RAW_POS_MAX)))

    def to_units(self, ticks):
        if self.tick_min == self.tick_max:
            return self.unit_min
        f = (ticks - self.tick_min) / (self.tick_max - self.tick_min)
        return clamp(self.unit_min + f * (self.unit_max - self.unit_min),
                     self.unit_min, self.unit_max)

    def fraction(self, value):
        """Position as a fraction of the axis range - the load model's error."""
        return (value - self.unit_min) / (self.unit_max - self.unit_min)


class DriverInterface(Node):

    def __init__(self):
        super().__init__('hand_driver_interface')
        self.tp = self.declare_parameter('topic_prefix', 'R_').value
        self.jp = self.declare_parameter('joint_prefix', 'r_').value
        base_id = self.declare_parameter('base_id', 30).value
        self.frequency = float(self.declare_parameter('frequency', 50.0).value)
        coupling = self.declare_parameter('finger_coupling', 'mimic').value
        self.couple_rl = self.declare_parameter('couple_ring_little', True).value
        self.output = self.declare_parameter(
            'output', 'motor_commands' if coupling == 'independent'
            else 'trajectory').value
        traj_topic = self.declare_parameter(
            'trajectory_topic', 'hand_controller/joint_trajectory').value
        # Commands stay this far inside the joint limits: dartsim pins joints
        # that park exactly ON a limit (see rh8d_coupling_node).
        self.margin = self.declare_parameter('limit_margin', 0.03).value
        self.tfs = self.declare_parameter('time_from_start', 0.08).value
        # device speed units (0..1023) per aligned-unit/s, as in
        # aligned_interface; 0 = ignore target_speed and move at full rate.
        self.speed_scale = float(self.declare_parameter('speed_scale', 512.0).value)
        # Telemetry the simulation has no physical source for.
        self.temperature = int(self.declare_parameter('temperature', 32).value)
        self.torque_limit = int(self.declare_parameter('torque_limit', 1023).value)
        self.current_limit = int(self.declare_parameter('current_limit', 1000).value)
        # A position servo's torque - and so its current - is proportional to
        # its tracking error. gz reports a near-zero joint force for
        # position-controlled joints, so the error is the usable load signal:
        # near zero in free motion, growing as soon as a finger is blocked.
        self.ma_per_error = float(
            self.declare_parameter('current_per_error', 3000.0).value)
        self.moving_threshold = int(
            self.declare_parameter('moving_threshold', 3).value)
        self.no_echo_ir = int(self.declare_parameter('no_echo_ir', 255).value)

        self._build_motors(base_id)
        self._setup_interfaces(traj_topic)

        self.meas = {}
        self.vel = {}
        self.palm_ir = self.no_echo_ir
        self.last_tick = None
        self._warned_stiffness = False

        self.create_timer(1.0 / self.frequency, self.tick)
        self.get_logger().info(
            f'driver interface up: {len(self.motors)} motor axes, '
            f'topic_prefix="{self.tp}", joint_prefix="{self.jp}", '
            f'{self.frequency:.0f} Hz, output={self.output} '
            '(tick calibration defaults are NOT hardware-verified)')

    # ── Motor table ──────────────────────────────────────────────────────────

    def _build_motors(self, base_id):
        m = self.margin
        self.motors = []
        self.main_board = (f'{self.jp}main_board', base_id)

        def ticks(axis):
            return (self.declare_parameter(f'calib.{axis}.tick_min', 0).value,
                    self.declare_parameter(f'calib.{axis}.tick_max', 4095).value)

        for axis, (lo, hi) in PASSTHROUGH.items():
            self.motors.append(_Motor(self.jp + axis, axis,
                                      base_id + BUS_ORDER.index(axis),
                                      lo, hi, ticks(axis), True))
        for axis in CHAINS:
            self.motors.append(_Motor(self.jp + axis, axis,
                                      base_id + BUS_ORDER.index(axis),
                                      0.0, 1.0, ticks(axis), False))
        self.by_name = {mo.name: mo for mo in self.motors}
        self.by_id = {mo.bus_id: mo for mo in self.motors}

        # Feedback source per motor: the phalanx chain (ring+little share one
        # tendon, so the ring chain alone speaks for it), or the axis itself.
        # travel matches rh8d_coupling_node's definition, so a closure of 1.0
        # commanded reads back as 1.0.
        self.chain = {}
        for axis, chains in CHAINS.items():
            joints = [(self.jp + s, u) for s, u in chains[0]]
            self.chain[self.jp + axis] = (joints, sum(u - m for _, u in joints))

        # Sim-side joint names each motor drives.
        if self.output == 'motor_commands':
            # the coupling node's motor axes; ring+little may be split there
            # even though the hardware always drives them from one motor
            self.out_names = {mo.name: [mo.name] for mo in self.motors}
            if not self.couple_rl:
                self.out_names[f'{self.jp}ring_little_flexion_joint'] = [
                    f'{self.jp}ring_flexion_joint', f'{self.jp}little_flexion_joint']
        else:
            # mimic mode: the leader joint of each finger, and the direct axes
            self.out_names = {}
            for mo in self.motors:
                if mo.direct:
                    self.out_names[mo.name] = [mo.name]
                else:
                    leader, upper = self.chain[mo.name][0][0]
                    self.out_names[mo.name] = [leader]
                    if not self.couple_rl and 'ring_little' in mo.key:
                        self.out_names[mo.name].append(
                            f'{self.jp}little_proximal_joint')

    def _setup_interfaces(self, traj_topic):
        tp = self.tp
        self.pub_joints = self.create_publisher(AllJoints, tp + 'Joints', 10)
        self.pub_boards = self.create_publisher(AllMainBoards, tp + 'Main_Boards', 10)
        self.create_subscription(JointListSetSpeedPos, tp + 'speed_position',
                                 self.on_speed_pos, 10)
        self.create_subscription(JointListSetStiffness, tp + 'stiffness',
                                 self.on_stiffness, 10)
        self.create_subscription(ClearHWError, tp + 'clear_error',
                                 self.on_clear_error, 10)
        self.create_subscription(SetShutdownCond, tp + 'shutdown_condition',
                                 self.on_shutdown, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 50)
        self.create_subscription(Range, 'palm_ir/range', self.on_palm_ir, 10)

        if self.output == 'motor_commands':
            self.cmd_pub = self.create_publisher(JointState, 'motor_commands', 10)
        else:
            self.cmd_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

    # ── Feedback ─────────────────────────────────────────────────────────────

    def on_joint_states(self, msg):
        self.meas.update(zip(msg.name, msg.position))
        if msg.velocity:
            self.vel.update(zip(msg.name, msg.velocity))

    def on_palm_ir(self, msg):
        self.palm_ir = int(clamp(round(msg.range * 1000.0), 0, 65535))

    def measured(self, motor):
        """Present value in the motor's aligned unit, or None if unknown."""
        if motor.direct:
            return self.meas.get(motor.name)
        joints, travel = self.chain[motor.name]
        seen = [self.meas[j] for j, _ in joints if j in self.meas]
        if not seen or travel <= 0:
            return None
        return clamp(sum(seen) / travel, 0.0, 1.0)

    def measured_rate(self, motor):
        if motor.direct:
            return self.vel.get(motor.name, 0.0)
        joints, travel = self.chain[motor.name]
        if travel <= 0:
            return 0.0
        return sum(self.vel.get(j, 0.0) for j, _ in joints) / travel

    # ── Command callbacks ────────────────────────────────────────────────────

    def _lookup(self, name):
        if name.isnumeric():
            return self.by_id.get(int(name))
        return self.by_name.get(name)

    def on_speed_pos(self, msg):
        for entry in msg.joints:
            motor = self._lookup(entry.name)
            if motor is None:
                self.get_logger().warn(
                    f'No mapping for joint "{entry.name}". Ignore if running '
                    '2 hands on different ports.')
                continue
            if motor.cmd is None:
                # First command: start from where the hand actually is, so the
                # rate limit produces a smooth move instead of a jump.
                now = self.measured(motor)
                motor.cmd = now if now is not None else motor.unit_min
            motor.target = motor.to_units(entry.target_pos)
            if entry.target_speed >= 0:
                motor.target_speed_raw = int(clamp(entry.target_speed,
                                                   0, RAW_SPEED_MAX))
                span = motor.unit_max - motor.unit_min
                if self.speed_scale <= 0 or motor.target_speed_raw == 0:
                    motor.rate = 0.0        # 0 ticks = maximum, as on hardware
                else:
                    motor.rate = motor.target_speed_raw / self.speed_scale * span
            # target_speed < 0 keeps the previous target speed, as on hardware.

    def on_stiffness(self, msg):
        now = time.time()
        for entry in msg.joints:
            motor = self._lookup(entry.name)
            if motor is None:
                self.get_logger().warn(f'No mapping for joint "{entry.name}"')
                continue
            if not 1 <= entry.stiffness <= 9:
                self.get_logger().warn(
                    f'Stiffness {entry.stiffness} out of range [1-9]')
                continue
            if (motor.time_last_stiffness is not None
                    and now - motor.time_last_stiffness < SECURITY_PERIOD):
                self.get_logger().warn(
                    f'Set stiffness on "{entry.name}" too frequent - discarded')
                continue
            motor.time_last_stiffness = now
            motor.stiffness = entry.stiffness
        if not self._warned_stiffness:
            self._warned_stiffness = True
            self.get_logger().warn(
                f'Stiffness is recorded and echoed back on {self.tp}Joints, but '
                'the simulated joints use the controller gains from the '
                'controllers yaml - it has no mechanical effect here.')

    def on_clear_error(self, msg):
        motor = self._lookup(msg.name)
        if motor is None:
            self.get_logger().warn(f'No mapping for joint "{msg.name}"')
            return
        now = time.time()
        if (motor.time_last_clear is not None
                and now - motor.time_last_clear < SECURITY_PERIOD):
            self.get_logger().warn(
                f'Clear error on "{msg.name}" too frequent - discarded')
            return
        motor.time_last_clear = now
        self.get_logger().info(f'CLEARING ERROR on "{msg.name}"')

    def on_shutdown(self, msg):
        motor = self._lookup(msg.name)
        if motor is None:
            self.get_logger().warn(f'No mapping for joint "{msg.name}"')
            return
        self.get_logger().info(
            f'SETTING SHUTDOWN CONDITIONS on "{msg.name}" '
            f'(temperature={msg.temperature} overload={msg.overload})')

    # ── Periodic loop ────────────────────────────────────────────────────────

    def tick(self):
        now = self.get_clock().now()
        dt = 1.0 / self.frequency
        if self.last_tick is not None:
            dt = max((now - self.last_tick).nanoseconds * 1e-9, 1e-4)
        self.last_tick = now

        self.publish_commands(dt)
        self.publish_joints(now)
        self.publish_main_boards(now)

    def sim_value(self, motor, value):
        """Aligned unit -> what the simulation's command topic expects."""
        if motor.direct:
            return clamp(value, motor.unit_min + self.margin,
                         motor.unit_max - self.margin)
        if self.output == 'motor_commands':
            return clamp(value, 0.0, 1.0)          # closure fraction
        _, upper = self.chain[motor.name][0][0]    # mimic leader joint
        return clamp(value, 0.0, 1.0) * (upper - self.margin)

    def publish_commands(self, dt):
        if not any(mo.cmd is not None for mo in self.motors):
            return  # nothing commanded yet - leave the sim to other publishers
        for mo in self.motors:
            if mo.cmd is None or mo.target is None:
                continue
            step = mo.rate * dt if mo.rate > 0 else abs(mo.target - mo.cmd)
            mo.cmd += clamp(mo.target - mo.cmd, -step, step)

        names, positions = [], []
        for mo in self.motors:
            value = mo.cmd
            if value is None:
                measured = self.measured(mo)
                value = measured if measured is not None else mo.unit_min
            for joint in self.out_names[mo.name]:
                names.append(joint)
                positions.append(self.sim_value(mo, value))

        if self.output == 'motor_commands':
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = names
            js.position = positions
            self.cmd_pub.publish(js)
        else:
            traj = JointTrajectory()
            traj.joint_names = names
            traj.points = [JointTrajectoryPoint(
                positions=positions,
                time_from_start=Duration(sec=int(self.tfs),
                                         nanosec=int((self.tfs % 1) * 1e9)))]
            self.cmd_pub.publish(traj)

    def publish_joints(self, now):
        msg = AllJoints()
        msg.header.stamp = now.to_msg()
        for mo in self.motors:
            lj = LoneJoint()
            lj.name = mo.name
            lj.bus_id = mo.bus_id
            lj.stiffness = mo.stiffness
            measured = self.measured(mo)
            lj.present_position = (mo.to_ticks(measured)
                                   if measured is not None else 0)
            span = mo.unit_max - mo.unit_min
            raw_speed = 0
            if self.speed_scale > 0 and span > 0:
                raw_speed = int(round(clamp(
                    self.measured_rate(mo) / span * self.speed_scale,
                    -RAW_SPEED_MAX, RAW_SPEED_MAX)))
            lj.present_speed = raw_speed
            lj.moving = 1 if abs(raw_speed) > self.moving_threshold else 0
            lj.target_position = (mo.to_ticks(mo.target)
                                  if mo.target is not None
                                  else lj.present_position)
            lj.target_speed = mo.target_speed_raw
            lj.torque_limit = self.torque_limit
            lj.temperature = self.temperature
            lj.hw_error_condition = 0
            current = 0
            if mo.cmd is not None and measured is not None:
                current = int(clamp(
                    (mo.fraction(mo.cmd) - mo.fraction(measured))
                    * self.ma_per_error, -32768, 32767))
            lj.current = current
            lj.stress_level = int(clamp(
                round(100.0 * abs(current) / max(self.current_limit, 1)), 0, 100))
            msg.joints.append(lj)
        msg.length = len(msg.joints)
        self.pub_joints.publish(msg)

    def publish_main_boards(self, now):
        name, bus_id = self.main_board
        board = LoneMainBoard()
        board.name = name
        board.id = bus_id
        board.palm_ir_sensor = self.palm_ir
        # The palm capacitive sensors have no simulated counterpart.
        board.capacitive_sensor_1 = 0
        board.capacitive_sensor_2 = 0
        msg = AllMainBoards()
        msg.header.stamp = now.to_msg()
        msg.boards.append(board)
        msg.length = 1
        self.pub_boards.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(DriverInterface())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
