#!/usr/bin/env python3
"""RH8D tendon coupling controller (for finger_coupling:=independent).

Maps one motor value per flexion tendon onto the three phalanx joints of each
finger, engaging them sequentially like the real underactuated hand: the
proximal joint consumes motor travel first, then the medial, then the distal.

With adaptive:=true the piecewise map becomes contact-aware, emulating the
tendon: if a phalanx stops following its command (blocked by an object), the
remaining motor travel is routed to the more distal joints, so the finger
wraps around the object instead of stalling - something the mimic-based
coupling modes structurally cannot do.

Interface
  in:  motor_commands (sensor_msgs/JointState) - any subset of the motor
       axes; unknown names are ignored. Motor names match the 'sequential'
       mode drive joints: <prefix>{thumb,index,middle}_flexion_joint,
       <prefix>ring_little_flexion_joint (or separate ring/little when
       couple_ring_little:=false), plus the pass-through axes
       <prefix>wrist_{rotation,adduction,flexion}_joint and
       <prefix>thumb_abduction_joint.
  in:  joint_states (sensor_msgs/JointState) - feedback for adaptive mode.
  out: output:=trajectory   -> JointTrajectory on hand_controller/joint_trajectory
       output:=joint_states -> JointState on joint_states (RViz demo, no sim)

A minimal "motor panel" URDF (one revolute joint per motor axis, correct
travel ranges, no geometry) is generated and latched on motor_description so
a joint_state_publisher GUI can offer one slider per motor.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def clamp(v, lo, hi):
    return min(max(v, lo), hi)


class CouplingNode(Node):

    def __init__(self):
        super().__init__('rh8d_coupling')
        p = self.declare_parameter('prefix', 'l_').value
        couple = self.declare_parameter('couple_ring_little', True).value
        self.adaptive = self.declare_parameter('adaptive', False).value
        self.tol = self.declare_parameter('blocked_tolerance', 0.12).value
        self.vel_tol = self.declare_parameter('blocked_velocity', 0.05).value
        rate = self.declare_parameter('rate', 25.0).value
        output = self.declare_parameter('output', 'trajectory').value
        traj_topic = self.declare_parameter(
            'trajectory_topic', 'hand_controller/joint_trajectory').value
        self.tfs = self.declare_parameter('time_from_start', 0.08).value
        # Commands stay this far inside the joint limits: dartsim pins joints
        # that park exactly ON a limit (they stay stuck until knocked free).
        self.margin = self.declare_parameter('limit_margin', 0.03).value

        finger = lambda f: [(f'{p}{f}_proximal_joint', 1.57),
                            (f'{p}{f}_medial_joint', 1.57),
                            (f'{p}{f}_distal_joint', 1.0)]
        thumb = [(f'{p}thumb_proximal_joint', 0.8),
                 (f'{p}thumb_medial_joint', 1.57),
                 (f'{p}thumb_distal_joint', 0.4)]
        # motor axis -> list of chains it drives (ring+little share one tendon)
        self.groups = {
            f'{p}thumb_flexion_joint': [thumb],
            f'{p}index_flexion_joint': [finger('index')],
            f'{p}middle_flexion_joint': [finger('middle')],
        }
        if couple:
            self.groups[f'{p}ring_little_flexion_joint'] = [finger('ring'), finger('little')]
        else:
            self.groups[f'{p}ring_flexion_joint'] = [finger('ring')]
            self.groups[f'{p}little_flexion_joint'] = [finger('little')]
        # direct axes with their limits
        self.passthrough = {
            f'{p}wrist_rotation_joint': (-1.5708, 1.5708),
            f'{p}wrist_adduction_joint': (-0.7854, 0.7854),
            f'{p}wrist_flexion_joint': (-0.7854, 0.7854),
            f'{p}thumb_abduction_joint': (-0.6, 0.7854),
        }
        self.motors = {name: 0.0 for name in
                       list(self.passthrough) + list(self.groups)}
        self.out_joints = list(self.passthrough) + [
            j for chains in self.groups.values() for c in chains for j, _ in c]
        self.meas = {}
        self.vel = {}

        self.create_subscription(JointState, 'motor_commands', self.on_cmd, 10)
        if self.adaptive:
            self.create_subscription(JointState, 'joint_states', self.on_feedback, 50)

        if output == 'trajectory':
            self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
            self.js_pub = None
        else:
            self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.traj_pub = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_pub = self.create_publisher(String, 'motor_description', qos)
        self.desc_pub.publish(String(data=self.motor_panel_urdf()))

        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'coupling {len(self.groups)} tendons -> {len(self.out_joints)} joints '
            f'(adaptive={self.adaptive})')

    def motor_panel_urdf(self):
        """URDF with one revolute joint per motor axis - slider panel source."""
        m = self.margin
        joints = [(n, (lo + m, hi - m)) for n, (lo, hi) in self.passthrough.items()] + [
            (motor, (0.0, sum(u - m for _, u in chains[0])))
            for motor, chains in self.groups.items()]
        parts = ['<robot name="rh8d_motors">', '<link name="motors"/>']
        for name, (lo, hi) in joints:
            parts += [f'<link name="{name}_pos"/>',
                      f'<joint name="{name}" type="revolute">',
                      f'<parent link="motors"/><child link="{name}_pos"/>',
                      '<axis xyz="0 0 1"/>',
                      f'<limit lower="{lo}" upper="{hi}" effort="1" velocity="1"/>',
                      '</joint>']
        parts.append('</robot>')
        return '\n'.join(parts)

    def on_cmd(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.motors:
                self.motors[name] = pos

    def on_feedback(self, msg):
        self.meas.update(zip(msg.name, msg.position))
        if msg.velocity:
            self.vel.update(zip(msg.name, msg.velocity))

    def solve_chain(self, chain, m):
        """Distribute motor travel m over the chain joints sequentially.

        Adaptive: a joint that lags its command by more than blocked_tolerance
        while barely moving (below blocked_velocity) is considered in contact;
        it only consumes the travel it actually achieved, the rest flows to
        the next joint. The velocity condition distinguishes real contact
        from ordinary tracking lag.
        """
        cmds = {}
        remaining = max(0.0, m)
        for joint, upper in chain:
            cmd = clamp(remaining, 0.0, upper - self.margin)
            if self.adaptive and joint in self.meas:
                achieved = self.meas[joint]
                blocked = (cmd - achieved > self.tol
                           and abs(self.vel.get(joint, 0.0)) < self.vel_tol)
                if blocked:  # in contact: stop pushing further
                    cmd = clamp(achieved + self.tol, 0.0, upper)
            cmds[joint] = cmd
            remaining -= cmd
        return cmds

    def tick(self):
        cmds = {}
        for name, (lo, hi) in self.passthrough.items():
            cmds[name] = clamp(self.motors[name], lo + self.margin, hi - self.margin)
        for motor, chains in self.groups.items():
            for chain in chains:
                cmds.update(self.solve_chain(chain, self.motors[motor]))

        now = self.get_clock().now().to_msg()
        if self.traj_pub is not None:
            traj = JointTrajectory()
            traj.joint_names = self.out_joints
            point = JointTrajectoryPoint(
                positions=[cmds[j] for j in self.out_joints],
                time_from_start=Duration(sec=int(self.tfs),
                                         nanosec=int((self.tfs % 1) * 1e9)))
            traj.points = [point]
            self.traj_pub.publish(traj)
        else:
            js = JointState()
            js.header.stamp = now
            js.name = self.out_joints
            js.position = [cmds[j] for j in self.out_joints]
            self.js_pub.publish(js)


def main():
    rclpy.init()
    try:
        rclpy.spin(CouplingNode())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
