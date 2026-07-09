#!/usr/bin/env python3
"""ARES-G marker-arm controller (L0.3 6.5).

Drives the 2-DOF paint-marker arm via the ``marker_position_controller``.
Publish ``std_msgs/Bool`` on ``/marker/deploy``:

    true  -> lower the arm to the ground-paint pose (mark a confirmed threat)
    false -> stow the arm back against the hull

Poses are simple, safe joint targets; the position controller interpolates.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray


class MarkerArm(Node):
    def __init__(self):
        super().__init__('marker_arm_node')
        # [shoulder, elbow] radians. Stowed = folded up; deployed = reaching down.
        self.declare_parameter('stow_pose', [1.4, -1.4])
        self.declare_parameter('deploy_pose', [-0.6, -0.3])

        self.stow = list(self.get_parameter('stow_pose').value)
        self.deploy = list(self.get_parameter('deploy_pose').value)

        self.pub = self.create_publisher(
            Float64MultiArray, '/marker_position_controller/commands', 10)
        self.create_subscription(Bool, '/marker/deploy', self._on_deploy, 10)

        # Start stowed after controllers are up.
        self.timer = self.create_timer(2.0, self._init_stow)
        self.get_logger().info('Marker arm ready (publish Bool on /marker/deploy).')

    def _init_stow(self):
        self.pub.publish(Float64MultiArray(data=self.stow))
        self.timer.cancel()

    def _on_deploy(self, msg):
        pose = self.deploy if msg.data else self.stow
        self.pub.publish(Float64MultiArray(data=pose))
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
