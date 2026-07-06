"""Simulate the buried-threat sensor suite from a ground-truth threat map.

As the rover drives, this node samples its belly-mounted sensors at the current
pose (from TF) and publishes:

  * /gpr/scan   (rover_msgs/GprScan)      - a GPR A-scan (amplitude vs depth). A
                buried point object contributes a reflection at slant range
                sqrt(depth^2 + offset^2); across successive scans this traces the
                characteristic hyperbola the processor later hunts for.
  * /sensors/reading (rover_msgs/SensorReading) - raw metal/magnetometer and VOC
                readings (range-attenuated + plume models) tagged with position.

This is a stand-in for real hardware so the whole detection/fusion pipeline can
be developed and evaluated with zero physical sensors.
"""
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros

from rover_msgs.msg import GprScan, SensorReading
from rover_detection import BuriedThreat, load_threats, lookup_xy, gaussian  # noqa: F401


class SensorSimNode(Node):
    def __init__(self):
        super().__init__('sensor_sim_node')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('gpr_depth_bins', 64)
        self.declare_parameter('gpr_max_depth', 1.0)
        self.declare_parameter('gpr_swath', 0.6)       # detectable half-width (m)
        self.declare_parameter('metal_r0', 0.4)        # metal falloff scale (m)
        self.declare_parameter('metal_range', 1.5)
        self.declare_parameter('voc_sigma', 0.8)       # plume std-dev (m)
        self.declare_parameter('noise_gpr', 0.05)
        self.declare_parameter('noise_metal', 0.03)
        self.declare_parameter('noise_voc', 0.03)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.nbins = int(self.get_parameter('gpr_depth_bins').value)
        self.max_depth = float(self.get_parameter('gpr_max_depth').value)
        self.dz = self.max_depth / self.nbins
        self.swath = float(self.get_parameter('gpr_swath').value)
        self.metal_r0 = float(self.get_parameter('metal_r0').value)
        self.metal_range = float(self.get_parameter('metal_range').value)
        self.voc_sigma = float(self.get_parameter('voc_sigma').value)
        self.noise_gpr = float(self.get_parameter('noise_gpr').value)
        self.noise_metal = float(self.get_parameter('noise_metal').value)
        self.noise_voc = float(self.get_parameter('noise_voc').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.threats = load_threats(self)
        self.get_logger().info(f'Ground-truth buried threats: {len(self.threats)}.')

        # Static horizontal ground layering (clutter that background subtraction removes).
        self._layers = np.zeros(self.nbins, dtype=np.float64)
        for depth_frac, amp in [(0.1, 0.4), (0.35, 0.25), (0.7, 0.15)]:
            b = int(depth_frac * self.nbins)
            if 0 <= b < self.nbins:
                self._layers[b] += amp

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_gpr = self.create_publisher(GprScan, '/gpr/scan', 10)
        self._pub_reading = self.create_publisher(SensorReading, '/sensors/reading', 20)

        self.create_timer(1.0 / max(rate, 1.0), self._sample)

    def _sample(self):
        now = self.get_clock().now()
        xy = lookup_xy(self._tf_buffer, self.map_frame, self.robot_frame, rclpy.time.Time())
        if xy is None:
            return
        x, y = xy
        self._emit_gpr(x, y, now)
        self._emit_metal(x, y, now)
        self._emit_voc(x, y, now)

    # --- GPR A-scan --------------------------------------------------------
    def _emit_gpr(self, x, y, now):
        depth_bins = (np.arange(self.nbins) + 0.5) * self.dz
        trace = self._layers.copy()
        for t in self.threats:
            offset = float(np.hypot(t.x - x, t.y - y))
            if offset > self.swath:
                continue
            slant = float(np.hypot(t.depth, offset))     # two-way -> depth-equivalent
            amp = (0.9 * t.size / max(t.size, 0.05)) * np.exp(-offset / self.swath)
            trace += amp * np.exp(-((depth_bins - slant) ** 2) / (2.0 * (1.5 * self.dz) ** 2))
        trace += np.random.normal(0.0, self.noise_gpr, self.nbins)

        msg = GprScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        msg.x = float(x)
        msg.y = float(y)
        msg.dz = float(self.dz)
        msg.trace = trace.astype(np.float32).tolist()
        self._pub_gpr.publish(msg)

    # --- Metal / magnetometer ---------------------------------------------
    def _emit_metal(self, x, y, now):
        value = 0.0
        for t in self.threats:
            r = float(np.hypot(t.x - x, t.y - y))
            if r > self.metal_range:
                continue
            value += t.metal / (1.0 + (r / self.metal_r0) ** 3)
        value += abs(np.random.normal(0.0, self.noise_metal))
        self._emit_reading('metal', x, y, min(value, 1.0), now)

    # --- VOC plume ---------------------------------------------------------
    def _emit_voc(self, x, y, now):
        value = 0.0
        for t in self.threats:
            r = float(np.hypot(t.x - x, t.y - y))
            value += t.voc * gaussian(r, self.voc_sigma)
        value += abs(np.random.normal(0.0, self.noise_voc))
        self._emit_reading('voc', x, y, min(value, 1.0), now)

    def _emit_reading(self, sensor, x, y, value, now):
        msg = SensorReading()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        msg.sensor = sensor
        msg.x = float(x)
        msg.y = float(y)
        msg.value = float(value)
        msg.depth = 0.0
        self._pub_reading.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
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
