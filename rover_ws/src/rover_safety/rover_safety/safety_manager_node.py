"""Supervisory safety and fault manager (Jetson side).

Sits in the actuation path: it subscribes to the high-level command (/cmd_vel
from Nav2 / mission / teleop) and republishes it to /cmd_vel_safe ONLY while the
system is healthy. On any fault it commands the guaranteed safe state (zero
velocity) instead. The hard-real-time watchdog + brake still lives on the MCU
(rover_hw_bridge/firmware); this layer adds system-level supervision.

Monitored faults
----------------
* Sensor dropout : LiDAR (/points) or IMU (/imu) rate collapses / times out.
* Motor stall    : a non-trivial command is issued but the wheels don't turn
                   (from /joint_states), for longer than a grace period.
* MCU heartbeat  : the emulated MCU stops sending /mcu/heartbeat (link/board dead).
* Emergency stop : /safety/estop latches an E-STOP until /safety/reset is called.

State machine
-------------
    NOMINAL   -> all clear, commands pass through.
    DEGRADED  -> a non-critical sensor dropped out; still drivable (dead-reckoning),
                 commands pass but speed is clamped.
    SAFE_STOP -> a critical fault; output forced to zero until it clears.
    ESTOP     -> latched; output zero until an explicit reset.
"""
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Bool, String, Empty
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

NOMINAL = 'NOMINAL'
DEGRADED = 'DEGRADED'
SAFE_STOP = 'SAFE_STOP'
ESTOP = 'ESTOP'


class RateWatch:
    """Tracks message arrival rate over a sliding window."""

    def __init__(self, window_s):
        self.window = window_s
        self.stamps = deque()

    def tick(self, now_ns):
        self.stamps.append(now_ns)
        horizon = now_ns - int(self.window * 1e9)
        while self.stamps and self.stamps[0] < horizon:
            self.stamps.popleft()

    def rate(self, now_ns):
        horizon = now_ns - int(self.window * 1e9)
        while self.stamps and self.stamps[0] < horizon:
            self.stamps.popleft()
        return len(self.stamps) / self.window

    def last_age_s(self, now_ns):
        if not self.stamps:
            return float('inf')
        return (now_ns - self.stamps[-1]) / 1e9


class SafetyManagerNode(Node):
    def __init__(self):
        super().__init__('safety_manager_node')

        self.declare_parameter('cmd_in_topic', '/cmd_vel')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_safe')
        self.declare_parameter('lidar_min_rate', 4.0)
        self.declare_parameter('imu_min_rate', 40.0)
        self.declare_parameter('heartbeat_timeout', 0.5)
        self.declare_parameter('stall_grace', 1.5)
        self.declare_parameter('stall_wheel_eps', 0.05)   # rad/s considered 'moving'
        self.declare_parameter('degraded_speed_scale', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.cmd_in = self.get_parameter('cmd_in_topic').value
        self.cmd_out = self.get_parameter('cmd_out_topic').value
        self.lidar_min = float(self.get_parameter('lidar_min_rate').value)
        self.imu_min = float(self.get_parameter('imu_min_rate').value)
        self.hb_timeout = float(self.get_parameter('heartbeat_timeout').value)
        self.stall_grace = float(self.get_parameter('stall_grace').value)
        self.stall_eps = float(self.get_parameter('stall_wheel_eps').value)
        self.degraded_scale = float(self.get_parameter('degraded_speed_scale').value)
        ctrl_rate = float(self.get_parameter('control_rate').value)

        self._lidar = RateWatch(2.0)
        self._imu = RateWatch(1.0)
        self._hb_last_ns = None
        self._last_cmd = Twist()
        self._last_cmd_ns = 0
        self._wheel_speed = 0.0
        self._stall_since_ns = None
        self._estop_latched = False
        self._state = NOMINAL

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(PointCloud2, '/points', self._on_lidar, sensor_qos)
        self.create_subscription(Imu, '/imu', self._on_imu, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(Empty, '/mcu/heartbeat', self._on_heartbeat, 10)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, 10)
        self.create_subscription(Twist, self.cmd_in, self._on_cmd, 10)

        self._pub_cmd = self.create_publisher(Twist, self.cmd_out, 10)
        self._pub_state = self.create_publisher(String, '/safety/state', 10)
        self._pub_ok = self.create_publisher(Bool, '/safety/ok', 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_service(Trigger, '/safety/reset', self._on_reset)

        self.create_timer(1.0 / max(ctrl_rate, 1.0), self._control)
        self.get_logger().info(
            f'Safety manager active: {self.cmd_in} -> {self.cmd_out} '
            '(gated on faults).')

    # --- Callbacks ---------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds

    def _on_lidar(self, _msg):
        self._lidar.tick(self._now())

    def _on_imu(self, _msg):
        self._imu.tick(self._now())

    def _on_joints(self, msg: JointState):
        if msg.velocity:
            self._wheel_speed = max(abs(v) for v in msg.velocity)

    def _on_heartbeat(self, _msg):
        self._hb_last_ns = self._now()

    def _on_estop(self, msg: Bool):
        if msg.data and not self._estop_latched:
            self._estop_latched = True
            self.get_logger().error('E-STOP asserted -> latched safe state.')

    def _on_cmd(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_ns = self._now()

    def _on_reset(self, _request, response):
        self._estop_latched = False
        self._stall_since_ns = None
        response.success = True
        response.message = 'E-STOP cleared; faults re-evaluated.'
        self.get_logger().warn('Safety reset requested; E-STOP cleared.')
        return response

    # --- Fault evaluation + command gate ----------------------------------
    def _evaluate(self):
        now = self._now()
        faults = []

        # Critical: IMU dropout, MCU heartbeat loss, e-stop.
        imu_rate = self._imu.rate(now)
        if imu_rate < self.imu_min:
            faults.append(('critical', f'IMU rate {imu_rate:.0f}<{self.imu_min:.0f}Hz'))
        if self._hb_last_ns is None or (now - self._hb_last_ns) / 1e9 > self.hb_timeout:
            faults.append(('critical', 'MCU heartbeat lost'))

        # Motor stall: commanded motion but wheels ~stationary beyond grace.
        commanding = (abs(self._last_cmd.linear.x) > 0.05 or
                      abs(self._last_cmd.angular.z) > 0.1)
        cmd_fresh = (now - self._last_cmd_ns) / 1e9 < 0.5
        if commanding and cmd_fresh and self._wheel_speed < self.stall_eps:
            if self._stall_since_ns is None:
                self._stall_since_ns = now
            elif (now - self._stall_since_ns) / 1e9 > self.stall_grace:
                faults.append(('critical', 'motor stall'))
        else:
            self._stall_since_ns = None

        # Non-critical: LiDAR dropout -> degraded (dead-reckoning still works).
        lidar_rate = self._lidar.rate(now)
        degraded = lidar_rate < self.lidar_min

        if self._estop_latched:
            return ESTOP, faults, lidar_rate, imu_rate
        if any(sev == 'critical' for sev, _ in faults):
            return SAFE_STOP, faults, lidar_rate, imu_rate
        if degraded:
            faults.append(('warn', f'LiDAR rate {lidar_rate:.0f}<{self.lidar_min:.0f}Hz'))
            return DEGRADED, faults, lidar_rate, imu_rate
        return NOMINAL, faults, lidar_rate, imu_rate

    def _control(self):
        state, faults, lidar_rate, imu_rate = self._evaluate()
        if state != self._state:
            level = self.get_logger().error if state in (SAFE_STOP, ESTOP) \
                else (self.get_logger().warn if state == DEGRADED
                      else self.get_logger().info)
            level(f'Safety state: {self._state} -> {state}')
            self._state = state

        out = Twist()
        if state in (NOMINAL, DEGRADED):
            out = self._last_cmd
            if state == DEGRADED:
                out.linear.x *= self.degraded_scale
                out.angular.z *= self.degraded_scale
        # SAFE_STOP / ESTOP -> out stays zero (guaranteed safe state).
        self._pub_cmd.publish(out)

        self._pub_state.publish(String(data=state))
        self._pub_ok.publish(Bool(data=(state in (NOMINAL, DEGRADED))))
        self._publish_diag(state, faults, lidar_rate, imu_rate)

    def _publish_diag(self, state, faults, lidar_rate, imu_rate):
        status = DiagnosticStatus()
        status.name = 'safety: system'
        status.hardware_id = 'rover'
        if state in (SAFE_STOP, ESTOP):
            status.level = DiagnosticStatus.ERROR
        elif state == DEGRADED:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = state
        status.values = [
            KeyValue(key='lidar_rate_hz', value=f'{lidar_rate:.1f}'),
            KeyValue(key='imu_rate_hz', value=f'{imu_rate:.1f}'),
            KeyValue(key='wheel_speed', value=f'{self._wheel_speed:.2f}'),
            KeyValue(key='estop_latched', value=str(self._estop_latched)),
            KeyValue(key='faults', value=';'.join(m for _, m in faults) or 'none'),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._pub_diag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManagerNode()
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
