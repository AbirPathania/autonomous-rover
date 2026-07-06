"""Threat exclusion-zone manager -> Nav2 KeepoutFilter mask.

When the mission layer confirms a buried threat, it publishes an exclusion
polygon here. This node rasterises all active polygons into an OccupancyGrid
"keepout" mask (100 = keep out, 0 = free) and republishes it (latched). Nav2's
KeepoutFilter marks those cells lethal in both costmaps, which invalidates any
path crossing them and forces the planner to reroute -- exactly the
"confirmed threats turned into exclusion polygons that force a replan" behaviour.

Interfaces
----------
Subscribes ``/threat/add_zone``  geometry_msgs/PolygonStamped
    Add an exclusion polygon (points assumed already in ``mask_frame``).
Service    ``/threat/clear``      std_srvs/Empty
    Remove all exclusion zones.
Publishes  ``/keepout_filter_mask`` nav_msgs/OccupancyGrid  (latched)
    The keepout mask consumed by the costmap KeepoutFilter.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from std_srvs.srv import Empty


def points_in_polygon(cx, cy, poly):
    """Vectorised ray-casting point-in-polygon test.

    cx, cy : 1-D arrays of cell-centre coordinates.
    poly   : (M, 2) array of polygon vertices.
    Returns a boolean array, True where the point is inside the polygon.
    """
    n = len(poly)
    inside = np.zeros(cx.shape, dtype=bool)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > cy) != (yj > cy)) & (
            cx < (xj - xi) * (cy - yi) / (yj - yi + 1e-12) + xi)
        inside ^= cond
        j = i
    return inside


class ThreatZoneNode(Node):
    """Maintains confirmed-threat exclusion polygons and publishes a keepout mask."""

    def __init__(self):
        super().__init__('threat_zone_node')

        self.declare_parameter('mask_frame', 'odom')
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('size_m', 40.0)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -20.0)

        self.mask_frame = self.get_parameter('mask_frame').value
        self.res = float(self.get_parameter('resolution').value)
        size_m = float(self.get_parameter('size_m').value)
        self.ox = float(self.get_parameter('origin_x').value)
        self.oy = float(self.get_parameter('origin_y').value)
        self.n = int(round(size_m / self.res))

        # Pre-compute cell-centre world coordinates (flattened, row-major).
        gx = self.ox + (np.arange(self.n) + 0.5) * self.res
        gy = self.oy + (np.arange(self.n) + 0.5) * self.res
        mx, my = np.meshgrid(gx, gy)  # (n, n), row index = y
        self._cx = mx.reshape(-1)
        self._cy = my.reshape(-1)

        self._polygons = []            # list of (M, 2) arrays
        self._mask = np.zeros(self.n * self.n, dtype=np.int8)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            OccupancyGrid, '/keepout_filter_mask', latched)
        self.create_subscription(
            PolygonStamped, '/threat/add_zone', self._on_add, 10)
        self.create_service(Empty, '/threat/clear', self._on_clear)

        self._publish_mask()
        self.get_logger().info(
            f'Threat keepout mask {self.n}x{self.n} @ {self.res} m in '
            f'frame "{self.mask_frame}". Awaiting /threat/add_zone.')

    def _on_add(self, msg: PolygonStamped):
        pts = msg.polygon.points
        if len(pts) < 3:
            self.get_logger().warn('Ignoring threat zone with < 3 vertices.')
            return
        if msg.header.frame_id and msg.header.frame_id != self.mask_frame:
            self.get_logger().warn(
                f'Threat zone frame "{msg.header.frame_id}" != mask frame '
                f'"{self.mask_frame}"; assuming coordinates are in the mask frame.')
        poly = np.array([[p.x, p.y] for p in pts], dtype=np.float64)
        self._polygons.append(poly)
        self._rasterise()
        self._publish_mask()
        self.get_logger().info(
            f'Added exclusion zone ({len(pts)} vertices); '
            f'{len(self._polygons)} active. Costmaps will replan.')

    def _on_clear(self, request, response):
        self._polygons.clear()
        self._mask[:] = 0
        self._publish_mask()
        self.get_logger().info('Cleared all exclusion zones.')
        return response

    def _rasterise(self):
        self._mask[:] = 0
        for poly in self._polygons:
            inside = points_in_polygon(self._cx, self._cy, poly)
            self._mask[inside] = 100

    def _publish_mask(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.mask_frame
        grid.info.resolution = self.res
        grid.info.width = self.n
        grid.info.height = self.n
        grid.info.origin.position.x = self.ox
        grid.info.origin.position.y = self.oy
        grid.info.origin.orientation.w = 1.0
        grid.data = self._mask.tolist()
        self._pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = ThreatZoneNode()
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
