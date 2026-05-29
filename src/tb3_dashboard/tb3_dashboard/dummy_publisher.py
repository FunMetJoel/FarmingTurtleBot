import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.pub_battery    = self.create_publisher(Float32,      '/dashboard/battery',             10)
        self.pub_water_rem  = self.create_publisher(Float32,      '/dashboard/water_remaining_pct', 10)
        self.pub_coverage   = self.create_publisher(Float32,      '/dashboard/coverage_pct',        10)
        self.pub_saved_pct  = self.create_publisher(Float32,      '/dashboard/water_saved_pct',     10)
        self.pub_water_dt   = self.create_publisher(Float32,      '/dashboard/water_dt_liters',     10)
        self.pub_water_naive= self.create_publisher(Float32,      '/dashboard/water_naive_liters',  10)
        self.pub_robot_pose = self.create_publisher(PoseStamped,  '/dashboard/robot_pose',          10)
        self.pub_rainy      = self.create_publisher(Bool,         '/twin/context/rainy',            10)
        self.pub_speed      = self.create_publisher(Float32,      '/twin/limits/speed_scale',       10)
        self.pub_alerts     = self.create_publisher(String,       '/twin/alerts',                   10)
        self.pub_map        = self.create_publisher(OccupancyGrid,'/map',                           10)
        self.pub_humidity   = self.create_publisher(OccupancyGrid,'/humidityMap',                   10)

        self._t = 0.0
        self._map     = self._build_map()
        self._humidity= self._build_humidity()

        self.create_timer(0.5,  self._tick)
        self.create_timer(15.0, self._alert)
        self.get_logger().info('Dummy publisher running — open http://localhost:8080')

    # ── per-tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        t = self._t
        self._t += 0.5

        self._f32(self.pub_battery,     max(0.0, 100.0 - t / 36.0))
        self._f32(self.pub_water_rem,   max(0.0, 100.0 - t / 3.0))
        self._f32(self.pub_coverage,    min(95.0, t * 1.5))
        saved = 60.0 + 15.0 * math.sin(t / 30.0)
        self._f32(self.pub_saved_pct,   saved)
        water_dt = 0.05 + t * 0.008
        self._f32(self.pub_water_dt,    water_dt)
        self._f32(self.pub_water_naive, water_dt / max(0.01, (100 - saved) / 100))

        # robot moves in a circle
        angle = t * 0.25
        r = 1.5
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 3.0 + r * math.cos(angle)
        pose.pose.position.y = 3.0 + r * math.sin(angle)
        heading = angle + math.pi / 2
        pose.pose.orientation.z = math.sin(heading / 2)
        pose.pose.orientation.w = math.cos(heading / 2)
        self.pub_robot_pose.publish(pose)

        rainy = (t % 60) > 45
        msg = Bool(); msg.data = rainy
        self.pub_rainy.publish(msg)
        self._f32(self.pub_speed, 0.6 if rainy else 1.0)

        # map + humidity at 1 Hz
        if int(t * 2) % 2 == 0:
            self._stamp(self._map)
            self._stamp(self._humidity)
            self.pub_map.publish(self._map)
            self.pub_humidity.publish(self._humidity)

    def _alert(self):
        msg = String()
        msg.data = f'Dummy alert at t={self._t:.0f}s'
        self.pub_alerts.publish(msg)

    # ── static map builders ───────────────────────────────────────────────────
    def _build_map(self):
        W, H, res = 80, 80, 0.1
        data = []
        for r in range(H):
            for c in range(W):
                if r == 0 or r == H-1 or c == 0 or c == W-1:
                    data.append(100)
                elif 25 <= r <= 30 and 10 <= c <= 45:
                    data.append(100)
                elif 50 <= r <= 55 and 35 <= c <= 70:
                    data.append(100)
                else:
                    data.append(0)
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.info.resolution = res
        g.info.width  = W
        g.info.height = H
        g.data = data
        return g

    def _build_humidity(self):
        W, H, res = 40, 40, 0.2
        data = []
        for r in range(H):
            for c in range(W):
                # wet patch top-left, dry bottom-right
                val = int(80 * (1 - c / W) * (1 - r / H) + 5)
                data.append(max(0, min(100, val)))
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.info.resolution = res
        g.info.width  = W
        g.info.height = H
        g.data = data
        return g

    # ── helpers ───────────────────────────────────────────────────────────────
    def _f32(self, pub, val):
        m = Float32(); m.data = float(val); pub.publish(m)

    def _stamp(self, grid):
        grid.header.stamp = self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
