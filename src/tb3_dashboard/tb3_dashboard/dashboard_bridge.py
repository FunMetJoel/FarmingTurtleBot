import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
import tf2_ros


class DashboardBridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')

        self.declare_parameter('humidity_dry_threshold', 40)
        self.declare_parameter('water_per_m2_liters', 0.5)

        self.dry_threshold = self.get_parameter('humidity_dry_threshold').value
        self.water_rate    = self.get_parameter('water_per_m2_liters').value

        self.pub_battery        = self.create_publisher(Float32,     '/dashboard/battery',             10)
        self.pub_water_saved_pct= self.create_publisher(Float32,     '/dashboard/water_saved_pct',     10)
        self.pub_water_dt       = self.create_publisher(Float32,     '/dashboard/water_dt_liters',     10)
        self.pub_water_naive    = self.create_publisher(Float32,     '/dashboard/water_naive_liters',  10)
        self.pub_coverage       = self.create_publisher(Float32,     '/dashboard/coverage_pct',        10)
        self.pub_robot_pose     = self.create_publisher(PoseStamped, '/dashboard/robot_pose',          10)

        self.create_subscription(OccupancyGrid, '/humidityMap', self._on_humidity_map, 10)
        self.create_subscription(OccupancyGrid, '/map',         self._on_map,          10)

        # Real robot publishes /battery_state; in simulation this never arrives
        # so we fall back to a time-based simulation
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery_state, sensor_qos)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._humidity_map  = None
        self._slam_map      = None
        self._battery_pct   = None  # None → use simulated fallback
        self._start_ns      = self.get_clock().now().nanoseconds

        self.create_timer(1.0, self._publish_all)

    def _on_humidity_map(self, msg):
        self._humidity_map = msg

    def _on_map(self, msg):
        self._slam_map = msg

    def _on_battery_state(self, msg: BatteryState):
        self._battery_pct = msg.percentage * 100.0

    def _publish_all(self):
        self._publish_battery()
        self._publish_coverage()
        self._publish_water_metrics()
        self._publish_robot_pose()

    def _publish_battery(self):
        if self._battery_pct is not None:
            pct = self._battery_pct
        else:
            elapsed_s = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
            pct = max(0.0, 100.0 - elapsed_s / 36.0)
        self._pub_f32(self.pub_battery, pct)

    def _publish_coverage(self):
        if self._slam_map is None:
            return
        data  = self._slam_map.data
        total = len(data)
        if total == 0:
            return
        known = sum(1 for c in data if c != -1)
        self._pub_f32(self.pub_coverage, known / total * 100.0)

    def _publish_water_metrics(self):
        if self._humidity_map is None:
            return
        data  = self._humidity_map.data
        known = [c for c in data if c != -1]
        if not known:
            return

        cell_area   = self._humidity_map.info.resolution ** 2
        total_cells = len(known)
        dry_cells   = sum(1 for c in known if c < self.dry_threshold)

        water_dt    = dry_cells   * cell_area * self.water_rate
        water_naive = total_cells * cell_area * self.water_rate
        saved_pct   = (1.0 - dry_cells / total_cells) * 100.0

        self._pub_f32(self.pub_water_saved_pct, saved_pct)
        self._pub_f32(self.pub_water_dt,        water_dt)
        self._pub_f32(self.pub_water_naive,     water_naive)

    def _publish_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = t.transform.translation.x
        pose.pose.position.y    = t.transform.translation.y
        pose.pose.position.z    = 0.0
        pose.pose.orientation   = t.transform.rotation
        self.pub_robot_pose.publish(pose)

    def _pub_f32(self, publisher, value):
        msg      = Float32()
        msg.data = float(value)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
