"""Software-in-the-loop MCU (STM32 / ODrive) emulator.

Stands in for the real microcontroller so the whole command path can be exercised
with no hardware. It mirrors what the firmware does in hard real time:

  * Receives VEL_CMD frames from the Jetson bridge (/mcu/rx).
  * Runs an INDEPENDENT WATCHDOG: if no fresh command arrives within
    watchdog_timeout, it commands the guaranteed safe state (zero velocity) and
    raises FAULT_WATCHDOG -- this is the last line of defence if the Jetson,
    safety node, or link dies.
  * Executes the (here idealised) closed-loop motor control and DRIVES THE SIM
    MOTORS by publishing /motor/cmd_vel to Gazebo.
  * Emits periodic HEARTBEAT and FEEDBACK frames back to the Jetson (/mcu/tx),
    including simulated wheel speeds, bus current, and fault flags.

On the real board this logic lives in a timer ISR at a fixed kHz rate; see
firmware/stm32 for the C reference.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray

from rover_hw_bridge import link_protocol as lp


class McuSimNode(Node):
    def __init__(self):
        super().__init__('mcu_sim_node')

        self.declare_parameter('watchdog_timeout', 0.2)   # s (hard-RT safety)
        self.declare_parameter('control_rate', 100.0)     # Hz
        self.declare_parameter('heartbeat_rate', 10.0)    # Hz
        self.declare_parameter('feedback_rate', 50.0)     # Hz
        self.declare_parameter('wheel_separation', 0.6)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('current_gain', 3.0)
        self.declare_parameter('overcurrent_a', 30.0)

        self.wd_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.sep = float(self.get_parameter('wheel_separation').value)
        self.radius = float(self.get_parameter('wheel_radius').value)
        self.k_current = float(self.get_parameter('current_gain').value)
        self.overcurrent = float(self.get_parameter('overcurrent_a').value)
        ctrl_rate = float(self.get_parameter('control_rate').value)
        hb_rate = float(self.get_parameter('heartbeat_rate').value)
        fb_rate = float(self.get_parameter('feedback_rate').value)

        self._parser = lp.StreamParser()
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_ns = 0
        self._fault = lp.FAULT_NONE
        self._v_left = 0.0
        self._v_right = 0.0
        self._current = 0.0
        self._start_ns = self.get_clock().now().nanoseconds

        self.create_subscription(UInt8MultiArray, '/mcu/rx', self._on_rx, 50)
        self._pub_motor = self.create_publisher(Twist, '/motor/cmd_vel', 10)
        self._pub_tx = self.create_publisher(UInt8MultiArray, '/mcu/tx', 50)

        self.create_timer(1.0 / ctrl_rate, self._control_loop)
        self.create_timer(1.0 / hb_rate, self._send_heartbeat)
        self.create_timer(1.0 / fb_rate, self._send_feedback)
        self.get_logger().info(
            f'MCU emulator online: watchdog {self.wd_timeout * 1000:.0f} ms, '
            f'control {ctrl_rate:.0f} Hz.')

    def _now(self):
        return self.get_clock().now().nanoseconds

    def _on_rx(self, msg: UInt8MultiArray):
        for decoded in self._parser.feed(bytes(bytearray(msg.data))):
            if decoded['type'] == 'vel_cmd':
                self._cmd_vx = decoded['vx']
                self._cmd_wz = decoded['wz']
                self._last_cmd_ns = self._now()

    def _control_loop(self):
        # Independent watchdog: stale command -> guaranteed safe state.
        age = (self._now() - self._last_cmd_ns) / 1e9
        if self._last_cmd_ns == 0 or age > self.wd_timeout:
            vx, wz = 0.0, 0.0
            self._fault |= lp.FAULT_WATCHDOG
        else:
            self._fault &= ~lp.FAULT_WATCHDOG
            vx, wz = self._cmd_vx, self._cmd_wz

        # Idealised closed-loop tracking -> drive the sim motors.
        out = Twist()
        out.linear.x = vx
        out.angular.z = wz
        self._pub_motor.publish(out)

        # Simulated per-side wheel speeds (rad/s) and bus current.
        self._v_left = (vx - wz * self.sep / 2.0) / self.radius
        self._v_right = (vx + wz * self.sep / 2.0) / self.radius
        self._current = self.k_current * (abs(vx) + abs(wz))
        if self._current > self.overcurrent:
            self._fault |= lp.FAULT_OVERCURRENT
        else:
            self._fault &= ~lp.FAULT_OVERCURRENT

    def _send_heartbeat(self):
        uptime_ms = int((self._now() - self._start_ns) / 1e6)
        self._emit(lp.pack_heartbeat(uptime_ms))

    def _send_feedback(self):
        self._emit(lp.pack_feedback(
            self._v_left, self._v_right, self._current, self._fault))

    def _emit(self, frame_bytes):
        out = UInt8MultiArray()
        out.data = list(frame_bytes)
        self._pub_tx.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = McuSimNode()
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
