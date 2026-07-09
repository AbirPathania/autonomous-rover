"""Terrain assessment: LiDAR point cloud -> drivability cost field.

Pipeline
--------
1. Accumulate incoming ``/points`` clouds into a persistent 2.5D elevation grid
   (fixed origin, auto-centred on the rover at start-up), tracking per cell the
   min / max / mean height and the point count.
2. Per cell derive two traversability signals:
     * roughness  = max_z - min_z within the cell (steps, rubble, ruts).
     * slope      = magnitude of the local elevation gradient (terrain tilt).
3. Fuse them into a single cost 0..100 using graded ramps, so "steep but
   passable" produces a high-but-finite cost the planner can pay to cross, while
   "will tip you over" (beyond the rover's slope/step limits) becomes lethal
   (100). Cells with too few points stay UNKNOWN (-1).

Published topics
----------------
``/terrain/costmap``    nav_msgs/OccupancyGrid   drivability cost (0..100, -1 unknown)
``/terrain/slope_deg``  nav_msgs/OccupancyGrid   debug: slope in degrees (scaled)
``/terrain/roughness``  nav_msgs/OccupancyGrid   debug: roughness in cm (scaled)

The costmap is consumed by Nav2 in Phase 4 (e.g. via a StaticLayer subscribed to
``/terrain/costmap``).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def quat_to_rot(x, y, z, w):
    """Return the 3x3 rotation matrix for a (x, y, z, w) quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class TerrainAnalysisNode(Node):
    """Builds and publishes a slope/roughness drivability costmap from LiDAR."""

    def __init__(self):
        super().__init__('terrain_analysis_node')

        # --- Parameters ---
        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('resolution', 0.2)     # metres / cell
        self.declare_parameter('size_m', 40.0)        # square map side, metres
        self.declare_parameter('range_max', 20.0)     # ignore points beyond this (m)
        self.declare_parameter('min_points', 2)       # cell known only above this
        self.declare_parameter('slope_warn_deg', 12.0)
        self.declare_parameter('slope_lethal_deg', 28.0)
        self.declare_parameter('step_warn_m', 0.06)
        self.declare_parameter('step_lethal_m', 0.22)
        self.declare_parameter('publish_rate_hz', 4.0)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.res = float(self.get_parameter('resolution').value)
        self.size_m = float(self.get_parameter('size_m').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.slope_warn = float(self.get_parameter('slope_warn_deg').value)
        self.slope_lethal = float(self.get_parameter('slope_lethal_deg').value)
        self.step_warn = float(self.get_parameter('step_warn_m').value)
        self.step_lethal = float(self.get_parameter('step_lethal_m').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.n = int(round(self.size_m / self.res))  # cells per side
        self._origin = None  # (ox, oy) set from first robot pose

        # Persistent accumulators (flattened row-major: index = iy * n + ix)
        cells = self.n * self.n
        self._cnt = np.zeros(cells, dtype=np.int32)
        self._zmin = np.full(cells, np.inf, dtype=np.float64)
        self._zmax = np.full(cells, -np.inf, dtype=np.float64)
        self._zsum = np.zeros(cells, dtype=np.float64)

        # --- TF ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._on_cloud, sensor_qos)
        self._pub_cost = self.create_publisher(
            OccupancyGrid, '/terrain/costmap', map_qos)
        self._pub_slope = self.create_publisher(
            OccupancyGrid, '/terrain/slope_deg', map_qos)
        self._pub_rough = self.create_publisher(
            OccupancyGrid, '/terrain/roughness', map_qos)

        self.create_timer(1.0 / max(rate, 0.5), self._publish)
        self.get_logger().info(
            f'Terrain analysis: {self.n}x{self.n} cells @ {self.res} m '
            f'({self.size_m} m) in frame "{self.map_frame}".')

    # ------------------------------------------------------------------ #
    def _lookup(self, target, source, stamp=None):
        """Return (R, t) transforming points from source frame to target frame.

        Uses the LATEST available transform (Time()) rather than the cloud's exact
        stamp. Under sim time the exact-stamp transform is frequently not yet in
        the buffer ("timestamp earlier than transform cache"), which would drop
        every cloud and leave the costmap empty (Nav2 then reports "robot out of
        bounds of the costmap"). Latest-transform is accurate enough for a slow
        ground rover and makes costmap publishing robust.
        """
        tf = self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2))
        q = tf.transform.rotation
        t = tf.transform.translation
        return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def _ensure_origin(self, stamp):
        """Centre the fixed grid on the rover the first time we have its pose."""
        if self._origin is not None:
            return True
        try:
            _, t = self._lookup(self.map_frame, self.robot_frame, stamp)
        except tf2_ros.TransformException:
            return False
        half = self.size_m / 2.0
        # Snap origin to the resolution grid.
        ox = math.floor((t[0] - half) / self.res) * self.res
        oy = math.floor((t[1] - half) / self.res) * self.res
        self._origin = (ox, oy)
        self.get_logger().info(f'Grid origin set to ({ox:.2f}, {oy:.2f}).')
        return True

    def _on_cloud(self, cloud):
        stamp = cloud.header.stamp
        if not self._ensure_origin(stamp):
            return
        try:
            R, t = self._lookup(self.map_frame, cloud.header.frame_id, stamp)
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'TF unavailable for cloud: {exc}', throttle_duration_sec=2.0)
            return

        pts = point_cloud2.read_points_list(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if not pts:
            return
        p = np.array([[q.x, q.y, q.z] for q in pts], dtype=np.float64)

        # Range gate in the sensor frame (before transform).
        rng = np.linalg.norm(p, axis=1)
        p = p[rng <= self.range_max]
        if p.shape[0] == 0:
            return

        # Transform to the map frame.
        p = p @ R.T + t

        ox, oy = self._origin
        ix = np.floor((p[:, 0] - ox) / self.res).astype(np.int64)
        iy = np.floor((p[:, 1] - oy) / self.res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        if not np.any(inside):
            return
        ix, iy, z = ix[inside], iy[inside], p[inside, 2]
        flat = iy * self.n + ix

        # Accumulate into the persistent grid.
        np.minimum.at(self._zmin, flat, z)
        np.maximum.at(self._zmax, flat, z)
        np.add.at(self._zsum, flat, z)
        np.add.at(self._cnt, flat, 1)

    # ------------------------------------------------------------------ #
    def _compute_grids(self):
        """Return (cost, slope_deg, roughness_m, valid) as (n, n) arrays."""
        n = self.n
        cnt = self._cnt.reshape(n, n)
        valid = cnt >= self.min_points

        with np.errstate(invalid='ignore', divide='ignore'):
            elev = np.where(cnt > 0, self._zsum.reshape(n, n) / np.maximum(cnt, 1), np.nan)
        roughness = np.where(
            valid, self._zmax.reshape(n, n) - self._zmin.reshape(n, n), 0.0)

        # Slope from central differences of elevation, only where both
        # neighbours are known (rows = y, cols = x).
        slope_deg = self._slope_from_elevation(elev, valid)

        step_cost = self._ramp(roughness, self.step_warn, self.step_lethal)
        slope_cost = self._ramp(slope_deg, self.slope_warn, self.slope_lethal)
        cost = np.maximum(step_cost, slope_cost)
        cost[roughness >= self.step_lethal] = 100.0
        cost[slope_deg >= self.slope_lethal] = 100.0
        return cost, slope_deg, roughness, valid

    def _slope_from_elevation(self, elev, valid):
        res = self.res
        slope = np.zeros_like(elev)

        def diff(axis):
            a_plus = np.roll(elev, -1, axis)
            a_minus = np.roll(elev, 1, axis)
            v_plus = np.roll(valid, -1, axis)
            v_minus = np.roll(valid, 1, axis)
            both = valid & v_plus & v_minus
            g = np.zeros_like(elev)
            g[both] = (a_plus[both] - a_minus[both]) / (2.0 * res)
            return g

        gy = diff(0)   # gradient along y (rows)
        gx = diff(1)   # gradient along x (cols)
        mag = np.sqrt(gx * gx + gy * gy)
        slope[valid] = np.degrees(np.arctan(mag[valid]))
        return slope

    @staticmethod
    def _ramp(value, lo, hi):
        """Linear 0->100 ramp between lo and hi (clamped)."""
        if hi <= lo:
            return np.where(value >= hi, 100.0, 0.0)
        return np.clip((value - lo) / (hi - lo), 0.0, 1.0) * 100.0

    # ------------------------------------------------------------------ #
    def _publish(self):
        if self._origin is None or not np.any(self._cnt):
            return
        cost, slope_deg, roughness, valid = self._compute_grids()

        cost_data = np.where(valid, np.round(cost), -1).astype(np.int8)
        # Debug grids scaled into 0..100 for RViz colour ramps.
        slope_scaled = np.where(
            valid, np.clip(slope_deg / self.slope_lethal * 100.0, 0, 100), -1)
        rough_scaled = np.where(
            valid, np.clip(roughness / self.step_lethal * 100.0, 0, 100), -1)

        self._pub_cost.publish(self._to_grid(cost_data))
        self._pub_slope.publish(self._to_grid(slope_scaled.astype(np.int8)))
        self._pub_rough.publish(self._to_grid(rough_scaled.astype(np.int8)))

    def _to_grid(self, data_2d):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.res
        grid.info.width = self.n
        grid.info.height = self.n
        grid.info.origin.position.x = self._origin[0]
        grid.info.origin.position.y = self._origin[1]
        grid.info.origin.orientation.w = 1.0
        grid.data = data_2d.reshape(-1).astype(np.int8).tolist()
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalysisNode()
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
