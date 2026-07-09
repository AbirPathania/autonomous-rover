#!/usr/bin/env python3
"""ARES-G marker-arm controller (L0.3 6.5).

Drives the 2-DOF paint-marker arm through the gazebo_ros joint_pose_trajectory
plugin. Publish ``std_msgs/Bool`` on ``/marker/deploy``:

    true  -> lower the arm to the ground-paint pose (mark a confirmed threat)
    false -> stow the arm back against the hull

Publishes a ``trajectory_msgs/JointTrajectory`` on ``/set_joint_trajectory``
naming only the two marker joints (the steering node owns the kingpins).
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


MARKER_JOINTS = ['marker_shoulder_joint', 'marker_elbow_joint']


class MarkerArm(Node):
    def __init__(self):
        super().__init__('marker_arm_node')
        # [shoulder, elbow] radians. Stowed = folded up; deployed = reaching down.
        self.declare_parameter('stow_pose', [1.4, -1.4])
        self.declare_parameter('deploy_pose', [-0.6, -0.3])
        self.declare_parameter('move_time', 0.8)

        self.stow = list(self.get_parameter('stow_pose').value)
        self.deploy = list(self.get_parameter('deploy_pose').value)
        self.move_time = self.get_parameter('move_time').value

        self.pub = self.create_publisher(
            JointTrajectory, '/set_joint_trajectory', 10)
        self.create_subscription(Bool, '/marker/deploy', self._on_deploy, 10)

        # Start stowed after the sim/plugin is up.
        self.timer = self.create_timer(2.0, self._init_stow)
        self.get_logger().info('Marker arm ready (publish Bool on /marker/deploy).')

    def _send(self, pose):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = MARKER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(pose)
        secs = int(self.move_time)
        pt.time_from_start = Duration(
            sec=secs, nanosec=int((self.move_time - secs) * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)

    def _init_stow(self):
        self._send(self.stow)
        self.timer.cancel()

    def _on_deploy(self, msg):
        pose = self.deploy if msg.data else self.stow
        self._send(pose)
        self.get_logger().info(
            f"Marker arm -> {'DEPLOY' if msg.data else 'STOW'} {pose}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
