import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.pub_battery    = self.create_publisher(Float32,      '/dashboard/battery',        10)
        self.pub_water_rem  = self.create_publisher(Float64,      '/rob_water_level',          10)
        self.pub_coverage   = self.create_publisher(Float32,      '/dashboard/coverage_pct',   10)
        self.pub_saved_pct  = self.create_publisher(Float32,      '/dashboard/water_saved_pct',10)
        self.pub_water_dt   = self.create_publisher(Float32,      '/dashboard/water_dt_liters',10)
        self.pub_water_naive= self.create_publisher(Float32,      '/dashboard/water_naive_liters', 10)
        self.pub_robot_pose = self.create_publisher(PoseStamped,  '/dashboard/robot_pose',     10)
        self.pub_rainy      = self.create_publisher(Bool,         '/twin/context/rainy',       10)
        self.pub_speed      = self.create_publisher(Float32,      '/twin/limits/speed_scale',  10)
        self.pub_alerts     = self.create_publisher(String,       '/twin/alerts',              10)
        self.pub_map        = self.create_publisher(OccupancyGrid,'/map',                      10)
        self.pub_humidity   = self.create_publisher(OccupancyGrid,'/humidityMap',              10)
        self.pub_irrigating = self.create_publisher(Bool,         '/irrigating',               10)

        self._t            = 0.0
        self._water_dt     = 0.0
        self._water_level  = 1.0
        self._prev_irr     = False
        self._prev_rainy   = False
        self._alerted_bat  = False
        self._alerted_wat  = False

        self._map              = self._build_map()
        self._humidity         = self._build_humidity()
        self._humidity_data    = list(self._humidity.data)
        self._hum_W            = self._humidity.info.width
        self._hum_H            = self._humidity.info.height
        self._hum_res          = self._humidity.info.resolution
        self._hum_ox           = self._humidity.info.origin.position.x
        self._hum_oy           = self._humidity.info.origin.position.y

        self.create_timer(0.5, self._tick)
        self.get_logger().info('Dummy publisher running — open http://localhost:8080')

    # ── per-tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        t = self._t
        self._t += 0.5

        angle   = t * 0.25
        r       = 1.5
        robot_x = 3.0 + r * math.cos(angle)
        robot_y = 3.0 + r * math.sin(angle)

        rainy     = (t % 60) > 45
        irrigating = ((angle % (2 * math.pi)) > math.pi) and not rainy

        # ── battery ──────────────────────────────────────────────────────────
        battery = max(0.0, 100.0 - t / 36.0)
        self._f32(self.pub_battery, battery)

        # ── water level — drains only when irrigating ─────────────────────
        if irrigating:
            self._water_level = max(0.0, self._water_level - 0.001)
        msg64 = Float64()
        msg64.data = self._water_level
        self.pub_water_rem.publish(msg64)

        # ── water usage metrics ───────────────────────────────────────────
        if irrigating:
            self._water_dt += 0.001
        saved = min(80.0, 30.0 + t * 0.2)
        self._f32(self.pub_saved_pct,   saved)
        self._f32(self.pub_water_dt,    self._water_dt)
        self._f32(self.pub_water_naive, self._water_dt * 2.5)

        # ── coverage ─────────────────────────────────────────────────────
        self._f32(self.pub_coverage, min(95.0, t * 1.5))

        # ── robot pose ────────────────────────────────────────────────────
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = robot_x
        pose.pose.position.y = robot_y
        heading = angle + math.pi / 2
        pose.pose.orientation.z = math.sin(heading / 2)
        pose.pose.orientation.w = math.cos(heading / 2)
        self.pub_robot_pose.publish(pose)

        # ── weather ───────────────────────────────────────────────────────
        msg_bool = Bool()
        msg_bool.data = rainy
        self.pub_rainy.publish(msg_bool)
        self._f32(self.pub_speed, 0.6 if rainy else 1.0)

        # ── irrigation status ─────────────────────────────────────────────
        msg_irr = Bool()
        msg_irr.data = irrigating
        self.pub_irrigating.publish(msg_irr)

        # ── humidity map — wet cells along robot path when irrigating ─────
        if irrigating:
            cx = int((robot_x - self._hum_ox) / self._hum_res)
            cy = int((robot_y - self._hum_oy) / self._hum_res)
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    idx = (cy + dr) * self._hum_W + (cx + dc)
                    if 0 <= idx < len(self._humidity_data):
                        self._humidity_data[idx] = min(100, self._humidity_data[idx] + 5)
            self._humidity.data = self._humidity_data

        # ── publish map + humidity at 1 Hz ────────────────────────────────
        if int(t * 2) % 2 == 0:
            self._stamp(self._map)
            self._stamp(self._humidity)
            self.pub_map.publish(self._map)
            self.pub_humidity.publish(self._humidity)

        # ── event-driven alerts ───────────────────────────────────────────
        if irrigating and not self._prev_irr:
            self._alert('Irrigation started')
        if not irrigating and self._prev_irr:
            self._alert('Irrigation complete')
        if rainy and not self._prev_rainy:
            self._alert('Rain detected — pausing irrigation')
        if not self._alerted_bat and battery < 20.0:
            self._alert(f'Battery low ({battery:.0f}%)')
            self._alerted_bat = True
        if not self._alerted_wat and self._water_level < 0.3:
            self._alert(f'Water level low ({self._water_level*100:.0f}%)')
            self._alerted_wat = True

        self._prev_irr   = irrigating
        self._prev_rainy = rainy

    # ── static map builders ───────────────────────────────────────────────────
    def _build_map(self):
        W, H, res = 80, 80, 0.1
        data = []
        for row in range(H):
            for col in range(W):
                if row == 0 or row == H-1 or col == 0 or col == W-1:
                    data.append(100)
                elif 25 <= row <= 30 and 10 <= col <= 45:
                    data.append(100)
                elif 50 <= row <= 55 and 35 <= col <= 70:
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
        ox, oy = 0.0, 0.0
        data = []
        for row in range(H):
            for col in range(W):
                # start dry (20-30) with slight gradient
                val = 20 + int(10 * col / W)
                data.append(val)
        from geometry_msgs.msg import Pose, Point, Quaternion
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.info.resolution = res
        g.info.width  = W
        g.info.height = H
        g.info.origin = Pose()
        g.info.origin.position = Point(x=ox, y=oy, z=0.0)
        g.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        g.data = data
        return g

    # ── helpers ───────────────────────────────────────────────────────────────
    def _f32(self, pub, val):
        m = Float32()
        m.data = float(val)
        pub.publish(m)

    def _stamp(self, grid):
        grid.header.stamp = self.get_clock().now().to_msg()

    def _alert(self, text):
        msg = String()
        msg.data = text
        self.pub_alerts.publish(msg)
        self.get_logger().info(f'Alert: {text}')


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
