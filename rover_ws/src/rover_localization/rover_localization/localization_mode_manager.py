"""LiDAR-health monitor and degraded dead-reckoning mode manager.

Watches the LiDAR point-cloud stream and decides whether the SLAM-corrected
global estimate can be trusted. When the LiDAR is blinded (snow, dust, smoke,
sensor fault) the point-cloud rate collapses; this node detects that and
announces DEGRADED_DEAD_RECKONING so that downstream consumers (mission logic,
Nav2 behaviours, operator UI) know the rover is coasting on the wheel+IMU EKF
(``ekf_local``) alone rather than on SLAM.

It is intentionally an *observer*: it does not reconfigure the EKFs at runtime.
The dual-EKF is already robust by construction -- ``ekf_map`` simply stops
receiving corrections when SLAM drops out, and ``ekf_local`` keeps publishing
odom -> base_footprint. This node makes that state observable and logged.

Published topics
----------------
``/localization/lidar_ok``   std_msgs/Bool      True while LiDAR rate is healthy.
``/localization/mode``       std_msgs/String    "SLAM_HEALTHY" | "DEGRADED_DEAD_RECKONING".
``/diagnostics``             diagnostic_msgs/DiagnosticArray
"""
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

MODE_HEALTHY = 'SLAM_HEALTHY'
MODE_DEGRADED = 'DEGRADED_DEAD_RECKONING'


class LocalizationModeManager(Node):
    """Monitors LiDAR throughput and publishes the active localization mode."""

    def __init__(self):
        super().__init__('localization_mode_manager')

        self.declare_parameter('lidar_topic', '/points')
        self.declare_parameter('min_rate_hz', 4.0)
        self.declare_parameter('window_sec', 2.0)
        self.declare_parameter('check_period_sec', 0.5)

        self._lidar_topic = self.get_parameter('lidar_topic').value
        self._min_rate = float(self.get_parameter('min_rate_hz').value)
        self._window = float(self.get_parameter('window_sec').value)
        check_period = float(self.get_parameter('check_period_sec').value)

        self._stamps = deque()
        self._mode = None  # force a transition log on first evaluation

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2, self._lidar_topic, self._on_cloud, sensor_qos)

        self._pub_ok = self.create_publisher(Bool, '/localization/lidar_ok', 10)
        self._pub_mode = self.create_publisher(String, '/localization/mode', 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_timer(check_period, self._evaluate)
        self.get_logger().info(
            f'Watching {self._lidar_topic}; healthy threshold '
            f'{self._min_rate:.1f} Hz over a {self._window:.1f}s window.')

    def _on_cloud(self, _msg):
        self._stamps.append(self.get_clock().now().nanoseconds)

    def _current_rate(self):
        now = self.get_clock().now().nanoseconds
        horizon = now - int(self._window * 1e9)
        while self._stamps and self._stamps[0] < horizon:
            self._stamps.popleft()
        return len(self._stamps) / self._window

    def _evaluate(self):
        rate = self._current_rate()
        healthy = rate >= self._min_rate
        mode = MODE_HEALTHY if healthy else MODE_DEGRADED

        if mode != self._mode:
            if mode == MODE_DEGRADED:
                self.get_logger().warn(
                    f'LiDAR rate {rate:.1f} Hz < {self._min_rate:.1f} Hz -> '
                    'entering DEGRADED_DEAD_RECKONING (wheel+IMU only).')
            else:
                self.get_logger().info(
                    f'LiDAR healthy at {rate:.1f} Hz -> SLAM correction trusted.')
            self._mode = mode

        self._pub_ok.publish(Bool(data=healthy))
        self._pub_mode.publish(String(data=mode))
        self._publish_diagnostics(healthy, rate)

    def _publish_diagnostics(self, healthy, rate):
        status = DiagnosticStatus()
        status.name = 'localization: LiDAR health'
        status.hardware_id = self._lidar_topic
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        status.message = MODE_HEALTHY if healthy else MODE_DEGRADED
        status.values = [
            KeyValue(key='rate_hz', value=f'{rate:.2f}'),
            KeyValue(key='min_rate_hz', value=f'{self._min_rate:.2f}'),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._pub_diag.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationModeManager()
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
