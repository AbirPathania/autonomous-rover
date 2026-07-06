"""Jetson-side hardware bridge.

Encodes the safety-gated velocity command into the framed link protocol and
sends it to the MCU; decodes the MCU's feedback + heartbeat frames back into ROS
topics. In SITL the "wire" is a UInt8 topic pair (/mcu/rx, /mcu/tx); swap these
for a pyserial or python-can endpoint on real hardware without touching the
framing.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, Empty, UInt8

from rover_hw_bridge import link_protocol as lp


class HwBridgeNode(Node):
    def __init__(self):
        super().__init__('hw_bridge_node')

        self.declare_parameter('cmd_topic', '/cmd_vel_safe')
        cmd_topic = self.get_parameter('cmd_topic').value

        self._parser = lp.StreamParser()

        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self.create_subscription(UInt8MultiArray, '/mcu/tx', self._on_mcu_bytes, 50)

        self._pub_to_mcu = self.create_publisher(UInt8MultiArray, '/mcu/rx', 10)
        self._pub_heartbeat = self.create_publisher(Empty, '/mcu/heartbeat', 10)
        self._pub_feedback = self.create_publisher(JointState, '/mcu/feedback', 10)
        self._pub_fault = self.create_publisher(UInt8, '/mcu/fault', 10)

        self.get_logger().info(f'HW bridge: {cmd_topic} -> MCU (framed link).')

    def _on_cmd(self, msg: Twist):
        frame = lp.pack_vel_cmd(float(msg.linear.x), float(msg.angular.z))
        out = UInt8MultiArray()
        out.data = list(frame)
        self._pub_to_mcu.publish(out)

    def _on_mcu_bytes(self, msg: UInt8MultiArray):
        for decoded in self._parser.feed(bytes(bytearray(msg.data))):
            if decoded['type'] == 'heartbeat':
                self._pub_heartbeat.publish(Empty())
            elif decoded['type'] == 'feedback':
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = ['left_wheels', 'right_wheels']
                js.velocity = [decoded['v_left'], decoded['v_right']]
                js.effort = [decoded['current'], decoded['current']]
                self._pub_feedback.publish(js)
                fault = decoded['fault']
                self._pub_fault.publish(UInt8(data=fault))
                if fault:
                    self.get_logger().warn(f'MCU fault flags: 0x{fault:02X}')


def main(args=None):
    rclpy.init(args=args)
    node = HwBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
