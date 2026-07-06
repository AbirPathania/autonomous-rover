"""GPR signal processing: background subtraction + hyperbola detection.

Buffers successive A-scans into a rolling B-scan (traces vs along-track
position). It then:

  1. Background subtraction -- removes the horizontally-coherent ground layering
     (subtract the per-depth-bin mean across the buffered scans), leaving
     localised reflectors.
  2. Hyperbola hunting -- a buried point object appears as a downward-opening
     hyperbola whose APEX (shallowest, strongest return) sits directly over the
     object. We take, per scan, the shallowest strong return, then find the apex
     as a local minimum of that curve that is flanked symmetrically by deeper
     returns (the hyperbola arms). That apex gives (x, depth, strength).

Emits a normalised /sensors/reading (sensor="gpr") at each confirmed apex.
"""
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from rover_msgs.msg import GprScan, SensorReading


class GprProcessorNode(Node):
    def __init__(self):
        super().__init__('gpr_processor_node')

        self.declare_parameter('window_scans', 25)     # B-scan width (scans)
        self.declare_parameter('min_scan_spacing', 0.05)  # m between kept scans
        self.declare_parameter('detect_threshold', 0.25)  # post-bg amplitude
        self.declare_parameter('min_arm_bins', 2)       # hyperbola arm depth drop
        self.declare_parameter('emit_period', 0.5)

        self.win = int(self.get_parameter('window_scans').value)
        self.spacing = float(self.get_parameter('min_scan_spacing').value)
        self.thresh = float(self.get_parameter('detect_threshold').value)
        self.min_arm = int(self.get_parameter('min_arm_bins').value)
        emit_period = float(self.get_parameter('emit_period').value)

        self._scans = deque(maxlen=self.win)  # (x, y, trace np.array)
        self._dz = 0.02
        self._frame = 'odom'

        self.create_subscription(GprScan, '/gpr/scan', self._on_scan, 20)
        self._pub = self.create_publisher(SensorReading, '/sensors/reading', 20)
        self.create_timer(emit_period, self._process)

    def _on_scan(self, msg: GprScan):
        self._dz = msg.dz if msg.dz > 0 else self._dz
        self._frame = msg.header.frame_id or self._frame
        trace = np.asarray(msg.trace, dtype=np.float64)
        if self._scans and np.hypot(msg.x - self._scans[-1][0],
                                    msg.y - self._scans[-1][1]) < self.spacing:
            return  # too close to the previous kept scan
        self._scans.append((msg.x, msg.y, trace))

    def _process(self):
        if len(self._scans) < max(5, self.win // 2):
            return
        xs = np.array([s[0] for s in self._scans])
        ys = np.array([s[1] for s in self._scans])
        nbins = min(len(s[2]) for s in self._scans)
        bscan = np.stack([s[2][:nbins] for s in self._scans], axis=0)  # (S, N)

        # 1) Background subtraction: remove per-bin mean (horizontal layering).
        bg = bscan.mean(axis=0, keepdims=True)
        clean = np.abs(bscan - bg)

        # Per scan: shallowest bin whose amplitude exceeds threshold.
        shallow = np.full(clean.shape[0], -1, dtype=int)
        strength = np.zeros(clean.shape[0])
        for i in range(clean.shape[0]):
            hits = np.where(clean[i] > self.thresh)[0]
            if hits.size:
                shallow[i] = hits[0]
                strength[i] = clean[i, hits[0]]

        # 2) Hyperbola apex = local minimum of the shallow-return curve flanked
        #    by deeper returns on both sides (the hyperbola arms).
        valid = shallow >= 0
        if valid.sum() < 3:
            return
        best = None
        for i in range(1, len(shallow) - 1):
            if not (valid[i] and valid[i - 1] and valid[i + 1]):
                continue
            apex = shallow[i]
            left = shallow[i - 1]
            right = shallow[i + 1]
            if (left - apex) >= self.min_arm and (right - apex) >= self.min_arm:
                score = strength[i]
                if best is None or score > best[0]:
                    best = (score, xs[i], ys[i], apex)

        if best is None:
            return
        score, ax, ay, apex_bin = best
        depth = apex_bin * self._dz
        value = float(np.clip(score, 0.0, 1.0))

        msg = SensorReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.sensor = 'gpr'
        msg.x = float(ax)
        msg.y = float(ay)
        msg.value = value
        msg.depth = float(depth)
        self._pub.publish(msg)
        self.get_logger().debug(
            f'GPR hyperbola apex @ ({ax:.2f},{ay:.2f}) depth {depth:.2f} m, '
            f'strength {value:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GprProcessorNode()
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
