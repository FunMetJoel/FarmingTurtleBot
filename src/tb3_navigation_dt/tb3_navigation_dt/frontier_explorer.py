import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener

MIN_FRONTIER_SIZE = 10  # cells — smaller clusters are noise


class FrontierExplorer(BasicNavigator):
    """BasicNavigator extended with map subscription and frontier detection."""

    def __init__(self):
        super().__init__('coverage_navigator')
        self.map_data: OccupancyGrid = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data = msg

    def get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def find_frontier_centroids(self):
        """Return list of (x, y, size) for each significant frontier cluster."""
        if self.map_data is None:
            return []

        h = self.map_data.info.height
        w = self.map_data.info.width
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y

        grid = np.array(self.map_data.data, dtype=np.int8).reshape(h, w)
        free = grid == 0

        # A frontier cell is free and has at least one unknown (-1) neighbour
        frontier_mask = np.zeros((h, w), dtype=bool)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbour = np.roll(grid, shift=(dr, dc), axis=(0, 1))
            frontier_mask |= free & (neighbour == -1)

        # np.roll wraps borders — clear them to avoid artefacts
        frontier_mask[[0, -1], :] = False
        frontier_mask[:, [0, -1]] = False

        # BFS to label connected frontier regions
        rows, cols = np.where(frontier_mask)
        visited = set()
        centroids = []

        for r, c in zip(rows.tolist(), cols.tolist()):
            if (r, c) in visited:
                continue
            cluster = []
            queue = [(r, c)]
            visited.add((r, c))
            while queue:
                cr, cc = queue.pop()
                cluster.append((cr, cc))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = cr + dr, cc + dc
                    key = (nr, nc)
                    if key not in visited and 0 <= nr < h and 0 <= nc < w and frontier_mask[nr, nc]:
                        visited.add(key)
                        queue.append(key)

            if len(cluster) >= MIN_FRONTIER_SIZE:
                cx = ox + (sum(p[1] for p in cluster) / len(cluster) + 0.5) * res
                cy = oy + (sum(p[0] for p in cluster) / len(cluster) + 0.5) * res
                centroids.append((cx, cy, len(cluster)))

        return centroids
