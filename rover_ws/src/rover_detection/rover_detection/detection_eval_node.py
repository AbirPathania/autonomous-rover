"""Detection evaluation: scores confirmed detections against ground truth.

The primary metric for buried-threat detection is NOT raw accuracy but the
false-alarm-rate per square metre (plus probability of detection). This node
knows the ground-truth threats (sim only), listens to confirmed detections on
/detection/threat, and reports:

  * Pd        = detected ground-truth threats / total ground-truth threats
  * FAR/m^2   = false alarms / area searched
  * counts    = true positives, false alarms, missed

Area searched is estimated from the rover's travelled path times the sensor
swath. Metrics are published on /detection/metrics and logged periodically.
"""
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from rover_detection import load_threats, lookup_xy


class DetectionEvalNode(Node):
    def __init__(self):
        super().__init__('detection_eval_node')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('match_radius', 1.0)   # TP if within this of a GT threat (m)
        self.declare_parameter('sensor_swath', 1.2)   # effective search width (m)
        self.declare_parameter('report_period', 5.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.match_r = float(self.get_parameter('match_radius').value)
        self.swath = float(self.get_parameter('sensor_swath').value)

        self.threats = load_threats(self)
        self._gt = np.array([[t.x, t.y] for t in self.threats], dtype=np.float64) \
            if self.threats else np.zeros((0, 2))
        self._gt_found = np.zeros(len(self.threats), dtype=bool)

        self._true_pos = 0
        self._false_alarms = 0
        self._path_len = 0.0
        self._last_xy = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(PointStamped, '/detection/threat', self._on_threat, 10)
        self._pub = self.create_publisher(String, '/detection/metrics', 10)
        self.create_timer(0.5, self._track_path)
        self.create_timer(float(self.get_parameter('report_period').value), self._report)

    def _track_path(self):
        xy = lookup_xy(self._tf_buffer, self.map_frame, self.robot_frame, rclpy.time.Time())
        if xy is None:
            return
        if self._last_xy is not None:
            self._path_len += float(np.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1]))
        self._last_xy = xy

    def _on_threat(self, msg: PointStamped):
        p = np.array([msg.point.x, msg.point.y])
        if len(self._gt):
            d = np.linalg.norm(self._gt - p, axis=1)
            j = int(np.argmin(d))
            if d[j] <= self.match_r:
                if not self._gt_found[j]:
                    self._gt_found[j] = True
                    self._true_pos += 1
                    self.get_logger().info(f'True positive on GT threat #{j}.')
                return
        self._false_alarms += 1
        self.get_logger().warn(
            f'False alarm at ({msg.point.x:.2f},{msg.point.y:.2f}).')

    def _report(self):
        n_gt = max(len(self.threats), 1)
        pd = self._gt_found.sum() / n_gt
        area = max(self._path_len * self.swath, 1e-3)
        far = self._false_alarms / area
        text = (f'Pd={pd:.2f} FAR/m2={far:.4f} '
                f'TP={self._true_pos} FA={self._false_alarms} '
                f'missed={len(self.threats) - int(self._gt_found.sum())} '
                f'area={area:.1f}m2')
        self._pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionEvalNode()
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
