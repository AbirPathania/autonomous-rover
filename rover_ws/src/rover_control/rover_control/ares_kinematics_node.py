#!/usr/bin/env python3
"""ARES-G four-wheel-steering (4WS) steering commander.

The wheels are driven (and wheel odometry published) by the gazebo_ros
diff-drive plugin. This node adds the genuine four-station steering on top: it
reads ``/cmd_vel`` and positions the four kingpins (L0.3 3.1.2) by publishing a
``trajectory_msgs/JointTrajectory`` on ``/set_joint_trajectory``, which the
gazebo_ros joint_pose_trajectory plugin applies.

  * Two front bogies steer as rigid units (shared kingpin) -> +delta.
  * Two rear wheels steer individually -> -delta (opposite-phase 4WS).
  * Near-zero forward speed -> kingpins centred (skid-steer fallback).

delta = atan( wz * L / (2 v) ), clamped to +/- max_steer.
"""
import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


STEER_JOINTS = ['left_bogie_steer_joint', 'right_bogie_steer_joint',
                'left_rear_steer_joint', 'right_rear_steer_joint']


class AresSteering(Node):
    def __init__(self):
        super().__init__('ares_kinematics_node')
        self.declare_parameter('wheelbase', 0.42)
        self.declare_parameter('max_steer', 0.6)
        self.declare_parameter('min_ackermann_speed', 0.06)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('settle_time', 0.15)

        self.L = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steer').value
        self.min_ack = self.get_parameter('min_ackermann_speed').value
        self.settle = self.get_parameter('settle_time').value
        rate = self.get_parameter('rate').value

        self._cmd = Twist()
        self.pub = self.create_publisher(
            JointTrajectory, '/set_joint_trajectory', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'ARES 4WS steering commander up (L={self.L}).')

    def _on_cmd(self, msg):
        self._cmd = msg

    def _steer_angles(self, vx, wz):
        if abs(vx) < self.min_ack:
            return [0.0, 0.0, 0.0, 0.0]            # skid-steer fallback
        delta = math.atan2(wz * self.L * 0.5, abs(vx))
        delta = max(-self.max_steer, min(self.max_steer, delta))
        # front bogies +delta, rear wheels -delta
        return [delta, delta, -delta, -delta]

    def _tick(self):
        angles = self._steer_angles(self._cmd.linear.x, self._cmd.angular.z)
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = STEER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = angles
        secs = int(self.settle)
        pt.time_from_start = Duration(
            sec=secs, nanosec=int((self.settle - secs) * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = AresSteering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
