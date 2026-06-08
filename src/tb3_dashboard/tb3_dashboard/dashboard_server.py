import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float32, Float64, Bool, String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
import tf2_ros


class DashboardServerNode(Node):
    def __init__(self):
        super().__init__('dashboard_server')

        self.declare_parameter('humidity_dry_threshold', 40)
        self.declare_parameter('water_per_m2_liters', 0.5)
        self.declare_parameter('http_port', 8080)

        self._dry_threshold = self.get_parameter('humidity_dry_threshold').value
        self._water_rate    = self.get_parameter('water_per_m2_liters').value
        port                = self.get_parameter('http_port').value

        self._lock  = threading.Lock()
        self._state = {
            'map':               None,
            'humidity_map':      None,
            'robot_pose':        None,
            'nav_path':          None,
            'battery':           100.0,
            'water_level':       None,
            'coverage_pct':      None,
            'water_saved_pct':   None,
            'water_dt_liters':   None,
            'water_naive_liters': None,
            'rainy':             None,
            'speed_scale':       None,
            'alerts':            [],
            'irrigating':        None,
        }

        self._slam_map     = None
        self._humidity_map = None
        self._battery_pct  = None   # None → time-based simulation
        self._start_ns     = self.get_clock().now().nanoseconds
        self._pose_fallback = None  # from /dashboard/robot_pose (dummy mode)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(OccupancyGrid, '/map',          self._on_map,          10)
        self.create_subscription(OccupancyGrid, '/humidityMap',  self._on_humidity_map, 10)
        self.create_subscription(Path,          '/plan',         self._on_nav_path,     10)
        self.create_subscription(BatteryState,  '/battery_state',self._on_battery,      sensor_qos)
        self.create_subscription(Float64,       '/rob_water_level',   self._on_water_level, 10)
        self.create_subscription(Bool,          '/twin/context/rainy',self._on_rainy,       10)
        self.create_subscription(Float32,       '/twin/limits/speed_scale', self._on_speed_scale, 10)
        self.create_subscription(String,        '/twin/alerts',  self._on_alert,        10)
        self.create_subscription(Bool,          '/irrigating',   self._on_irrigating,   10)
        # Fallback pose used by dummy_publisher (no TF in that mode)
        self.create_subscription(PoseStamped,   '/dashboard/robot_pose', self._on_pose_fallback, 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_timer(1.0, self._update_computed)

        self._start_http(port)

    # ── topic callbacks ───────────────────────────────────────────────────────

    def _on_map(self, msg):
        self._slam_map = msg
        with self._lock:
            self._state['map'] = _grid_to_dict(msg)

    def _on_humidity_map(self, msg):
        self._humidity_map = msg
        with self._lock:
            self._state['humidity_map'] = _grid_to_dict(msg)

    def _on_nav_path(self, msg):
        poses = [{'pose': {'position': {'x': p.pose.position.x, 'y': p.pose.position.y}}}
                 for p in msg.poses]
        with self._lock:
            self._state['nav_path'] = {'poses': poses}

    def _on_battery(self, msg: BatteryState):
        self._battery_pct = msg.percentage * 100.0

    def _on_water_level(self, msg):
        with self._lock:
            self._state['water_level'] = msg.data

    def _on_rainy(self, msg):
        with self._lock:
            self._state['rainy'] = msg.data

    def _on_speed_scale(self, msg):
        with self._lock:
            self._state['speed_scale'] = msg.data

    def _on_alert(self, msg):
        with self._lock:
            self._state['alerts'].append(msg.data)
            if len(self._state['alerts']) > 30:
                self._state['alerts'].pop(0)

    def _on_irrigating(self, msg):
        with self._lock:
            self._state['irrigating'] = msg.data

    def _on_pose_fallback(self, msg: PoseStamped):
        self._pose_fallback = msg

    # ── 1 Hz computed updates ─────────────────────────────────────────────────

    def _update_computed(self):
        self._update_battery()
        self._update_coverage()
        self._update_water_metrics()
        self._update_robot_pose()

    def _update_battery(self):
        if self._battery_pct is not None:
            pct = self._battery_pct
        else:
            elapsed = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
            pct = max(0.0, 100.0 - elapsed / 36.0)
        with self._lock:
            self._state['battery'] = pct

    def _update_coverage(self):
        if self._slam_map is None:
            return
        data  = self._slam_map.data
        total = len(data)
        if total == 0:
            return
        known = sum(1 for c in data if c != -1)
        with self._lock:
            self._state['coverage_pct'] = known / total * 100.0

    def _update_water_metrics(self):
        if self._humidity_map is None:
            return
        data  = self._humidity_map.data
        known = [c for c in data if c != -1]
        if not known:
            return
        cell_area   = self._humidity_map.info.resolution ** 2
        total_cells = len(known)
        dry_cells   = sum(1 for c in known if c < self._dry_threshold)
        with self._lock:
            self._state['water_dt_liters']   = dry_cells   * cell_area * self._water_rate
            self._state['water_naive_liters'] = total_cells * cell_area * self._water_rate
            self._state['water_saved_pct']   = (1.0 - dry_cells / total_cells) * 100.0

    def _update_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            pose = {
                'pose': {
                    'position':    {'x': t.transform.translation.x,
                                    'y': t.transform.translation.y, 'z': 0.0},
                    'orientation': {'x': t.transform.rotation.x,
                                    'y': t.transform.rotation.y,
                                    'z': t.transform.rotation.z,
                                    'w': t.transform.rotation.w},
                }
            }
            with self._lock:
                self._state['robot_pose'] = pose
            return
        except Exception:
            pass

        # Fall back to the /dashboard/robot_pose topic (dummy mode)
        fb = self._pose_fallback
        if fb is not None:
            p = fb.pose
            pose = {
                'pose': {
                    'position':    {'x': p.position.x, 'y': p.position.y, 'z': 0.0},
                    'orientation': {'x': p.orientation.x, 'y': p.orientation.y,
                                    'z': p.orientation.z, 'w': p.orientation.w},
                }
            }
            with self._lock:
                self._state['robot_pose'] = pose

    # ── HTTP server ───────────────────────────────────────────────────────────

    def _start_http(self, port: int):
        web_dir  = os.path.join(get_package_share_directory('tb3_dashboard'), 'web')
        node_ref = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    path = os.path.join(web_dir, 'index.html')
                    with open(path, 'rb') as f:
                        body = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == '/api/state':
                    with node_ref._lock:
                        snapshot = dict(node_ref._state)
                    body = json.dumps(snapshot).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, *_):
                pass

        server = HTTPServer(('0.0.0.0', port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Dashboard: http://localhost:{port}')


# ── helpers ───────────────────────────────────────────────────────────────────

def _grid_to_dict(msg):
    return {
        'info': {
            'width':      msg.info.width,
            'height':     msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {
                'position': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z,
                },
                'orientation': {
                    'x': msg.info.origin.orientation.x,
                    'y': msg.info.origin.orientation.y,
                    'z': msg.info.origin.orientation.z,
                    'w': msg.info.origin.orientation.w,
                },
            },
        },
        'data': list(msg.data),
    }


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
