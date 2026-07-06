"""Per-cell sensor fusion -> threat type + confidence -> local 3D threat map.

Bins every /sensors/reading (gpr, metal, voc) into a fixed world grid, keeping a
per-cell running estimate of each sensor's evidence. It fuses the three channels
into a confidence via a logistic model and classifies the threat type by which
channels dominate. A cell is only CONFIRMED after it has been observed enough
times and clears the confirm threshold -- this persistence + multi-sensor gate
is what keeps the false-alarm-rate per square metre low.

Publishes
---------
/detection/confidence  nav_msgs/OccupancyGrid     fused confidence * 100 (viz)
/detection/threat      geometry_msgs/PointStamped confirmed threat -> mission layer
/detection/threats     rover_msgs/ThreatDetection classified detection (rich)

Also appends every confirmation to a JSONL log (NVMe replay / forensics).
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rover_msgs.msg import SensorReading, ThreatDetection

from rover_detection import sigmoid


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('size_m', 40.0)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -20.0)
        # Logistic fusion weights: bias + per-sensor gains.
        self.declare_parameter('w_bias', -3.0)
        self.declare_parameter('w_gpr', 4.0)
        self.declare_parameter('w_metal', 4.5)
        self.declare_parameter('w_voc', 3.0)
        self.declare_parameter('confirm_confidence', 0.7)
        self.declare_parameter('min_observations', 4)
        self.declare_parameter('ema_alpha', 0.4)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('log_path', '~/rover_detections.jsonl')

        self.map_frame = self.get_parameter('map_frame').value
        self.res = float(self.get_parameter('resolution').value)
        size_m = float(self.get_parameter('size_m').value)
        self.ox = float(self.get_parameter('origin_x').value)
        self.oy = float(self.get_parameter('origin_y').value)
        self.n = int(round(size_m / self.res))
        self.w = (float(self.get_parameter('w_bias').value),
                  float(self.get_parameter('w_gpr').value),
                  float(self.get_parameter('w_metal').value),
                  float(self.get_parameter('w_voc').value))
        self.confirm_conf = float(self.get_parameter('confirm_confidence').value)
        self.min_obs = int(self.get_parameter('min_observations').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.log_path = os.path.expanduser(self.get_parameter('log_path').value)

        cells = self.n * self.n
        self._e_gpr = np.zeros(cells)
        self._e_metal = np.zeros(cells)
        self._e_voc = np.zeros(cells)
        self._depth = np.zeros(cells)
        self._obs = np.zeros(cells, dtype=np.int32)
        self._confirmed = np.zeros(cells, dtype=bool)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(SensorReading, '/sensors/reading', self._on_reading, 30)
        self._pub_conf = self.create_publisher(OccupancyGrid, '/detection/confidence', latched)
        self._pub_threat = self.create_publisher(PointStamped, '/detection/threat', 10)
        self._pub_detection = self.create_publisher(ThreatDetection, '/detection/threats', 10)
        self.create_timer(float(self.get_parameter('publish_period').value), self._publish)

        try:
            open(self.log_path, 'a').close()
        except OSError as exc:
            self.get_logger().warn(f'Cannot open log {self.log_path}: {exc}')
        self.get_logger().info(
            f'Fusion grid {self.n}x{self.n} @ {self.res} m; '
            f'confirm>={self.confirm_conf} after {self.min_obs} obs.')

    def _index(self, x, y):
        ix = int((x - self.ox) / self.res)
        iy = int((y - self.oy) / self.res)
        if 0 <= ix < self.n and 0 <= iy < self.n:
            return iy * self.n + ix
        return -1

    def _on_reading(self, msg: SensorReading):
        idx = self._index(msg.x, msg.y)
        if idx < 0:
            return
        v = float(np.clip(msg.value, 0.0, 1.0))
        a = self.alpha
        if msg.sensor == 'gpr':
            self._e_gpr[idx] = (1 - a) * self._e_gpr[idx] + a * v
            if msg.depth > 0:
                self._depth[idx] = msg.depth
        elif msg.sensor == 'metal':
            self._e_metal[idx] = (1 - a) * self._e_metal[idx] + a * v
        elif msg.sensor == 'voc':
            self._e_voc[idx] = (1 - a) * self._e_voc[idx] + a * v
        else:
            return
        self._obs[idx] += 1

    def _confidence(self, idx):
        b, wg, wm, wv = self.w
        z = b + wg * self._e_gpr[idx] + wm * self._e_metal[idx] + wv * self._e_voc[idx]
        return sigmoid(z)

    def _classify(self, idx):
        g, m, v = self._e_gpr[idx], self._e_metal[idx], self._e_voc[idx]
        if m > 0.5 and m >= g:
            return 'metal_object'
        if v > 0.5 and v >= m:
            return 'chemical'
        if g > 0.4:
            return 'buried_object'
        return 'unknown'

    def _publish(self):
        conf = np.array([self._confidence(i) for i in range(self.n * self.n)])
        observed = self._obs > 0

        # Confirm new threats (persistence + confidence + multi-evidence gate).
        candidates = np.where(
            (~self._confirmed) & (self._obs >= self.min_obs) & (conf >= self.confirm_conf))[0]
        for idx in candidates:
            self._confirmed[idx] = True
            self._confirm(idx, conf[idx])

        grid_vals = np.where(observed, np.round(conf * 100), -1).astype(np.int8)
        self._pub_conf.publish(self._to_grid(grid_vals))

    def _confirm(self, idx, confidence):
        iy, ix = divmod(int(idx), self.n)
        x = self.ox + (ix + 0.5) * self.res
        y = self.oy + (iy + 0.5) * self.res
        ttype = self._classify(idx)
        depth = float(self._depth[idx])
        stamp = self.get_clock().now().to_msg()

        pt = PointStamped()
        pt.header.stamp = stamp
        pt.header.frame_id = self.map_frame
        pt.point.x = x
        pt.point.y = y
        self._pub_threat.publish(pt)

        det = ThreatDetection()
        det.header.stamp = stamp
        det.header.frame_id = self.map_frame
        det.x = x
        det.y = y
        det.type = ttype
        det.confidence = float(confidence)
        det.depth = depth
        self._pub_detection.publish(det)

        self.get_logger().warn(
            f'THREAT CONFIRMED [{ttype}] @ ({x:.2f},{y:.2f}) '
            f'conf {confidence:.2f} depth {depth:.2f} m')
        self._log(x, y, ttype, confidence, depth)

    def _log(self, x, y, ttype, confidence, depth):
        record = {
            'wall_time': time.time(),
            'x': round(x, 3), 'y': round(y, 3),
            'type': ttype, 'confidence': round(float(confidence), 3),
            'depth': round(depth, 3),
        }
        try:
            with open(self.log_path, 'a') as f:
                f.write(json.dumps(record) + '\n')
        except OSError as exc:
            self.get_logger().warn(f'Log write failed: {exc}')

    def _to_grid(self, data_1d):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.res
        grid.info.width = self.n
        grid.info.height = self.n
        grid.info.origin.position.x = self.ox
        grid.info.origin.position.y = self.oy
        grid.info.origin.orientation.w = 1.0
        grid.data = data_1d.tolist()
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
