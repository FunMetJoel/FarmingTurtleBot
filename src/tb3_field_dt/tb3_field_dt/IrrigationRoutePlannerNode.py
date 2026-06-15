import math
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class IrrigationNode:
    id: str
    x: float
    y: float
    moisture: float
    moisture_drop_per_second: float


def nodes_from_humidity_map(msg, moisture_drop_per_second):
    width = msg.info.width
    height = msg.info.height

    if width * height != len(msg.data):
        raise ValueError('Humidity map dimensions do not match its data')

    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    nodes = []

    for index, moisture in enumerate(msg.data):
        if moisture < 0:
            continue

        row, column = divmod(index, width)
        nodes.append(IrrigationNode(
            id=f'{row}:{column}',
            x=origin_x + (column + 0.5) * resolution,
            y=origin_y + (row + 0.5) * resolution,
            moisture=float(moisture),
            moisture_drop_per_second=moisture_drop_per_second,
        ))

    return nodes


class IrrigationRoutePlannerNode(Node):

    def __init__(self):
        super().__init__('IrrigationRoutePlanner')

        self.declare_parameter('moisture_threshold', 40.0)
        self.declare_parameter('minimum_dry_nodes', 5)
        self.declare_parameter('average_drive_speed', 0.15)
        self.declare_parameter('irrigation_time_per_node', 6.0)
        self.declare_parameter('extra_time_factor', 1.20)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('moisture_drop_per_second', 0.1)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.reported_missing_transform = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.route_publisher = self.create_publisher(Path, '/irrigation/route', 10)
        self.debug_publisher = self.create_publisher(String, '/irrigation/route_debug', 10)
        self.create_subscription(
            OccupancyGrid,
            '/humidityMap',
            self.humidity_map_callback,
            10
        )

        self.nodes = []
        self.route_published = False
        self.timer = self.create_timer(1.0, self.plan_and_publish_route)

        self.get_logger().info('Irrigation route planner started')

    def humidity_map_callback(self, msg):
        moisture_drop = float(
            self.get_parameter('moisture_drop_per_second').value
        )

        try:
            self.nodes = nodes_from_humidity_map(msg, moisture_drop)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return

        self.frame_id = msg.header.frame_id or self.frame_id
        self.reported_missing_transform = False
        self.get_logger().info(
            f'Received {len(self.nodes)} humidity nodes in {self.frame_id}'
        )

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                'base_footprint',
                Time()
            )
        except TransformException:
            if not self.reported_missing_transform:
                self.get_logger().warning(
                    f'Waiting for transform '
                    f'{self.frame_id} -> base_footprint'
                )
                self.reported_missing_transform = True
            return None

        self.reported_missing_transform = False
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def plan_and_publish_route(self):
        if self.route_published:
            return

        if not self.nodes:
            return

        start = self.get_robot_position()
        if start is None:
            return

        threshold = float(self.get_parameter('moisture_threshold').value)
        minimum_dry_nodes = int(self.get_parameter('minimum_dry_nodes').value)
        extra_time_factor = float(self.get_parameter('extra_time_factor').value)

        dry_nodes = [node for node in self.nodes if node.moisture <= threshold]

        if len(dry_nodes) < minimum_dry_nodes:
            self.debug_publisher.publish(String(
                data=f'Waiting: {len(dry_nodes)} dry nodes, need {minimum_dry_nodes}'
            ))
            return

        route = self.nearest_neighbour_route(start, dry_nodes)
        dry_route_names = ' -> '.join([node.id for node in route])
        base_time = self.route_time(start, route)
        max_time = base_time * extra_time_factor

        candidates = [node for node in self.nodes if node not in route]
        route = self.add_extra_nodes(start, route, candidates, max_time, threshold)

        final_time = self.route_time(start, route)
        self.publish_route(route)

        route_names = ' -> '.join([node.id for node in route])
        debug_text = (
            f'Dry route: {dry_route_names} | final route: {route_names} | '
            f'base={base_time:.1f}s max={max_time:.1f}s final={final_time:.1f}s'
        )
        self.debug_publisher.publish(String(data=debug_text))
        self.get_logger().info(debug_text)
        self.get_logger().info('Route published once. Planner is now idle.')

        self.route_published = True
        self.timer.cancel()

    def nearest_neighbour_route(self, start, nodes):
        route = []
        unused = list(nodes)
        current = start

        while len(unused) > 0:
            closest = min(unused, key=lambda node: self.distance(current, (node.x, node.y)))
            route.append(closest)
            unused.remove(closest)
            current = (closest.x, closest.y)

        return route

    def add_extra_nodes(self, start, route, candidates, max_time, threshold):
        added_something = True

        while added_something:
            added_something = False
            best_route = None
            best_node = None
            best_time = None

            for node in candidates:
                for insert_index in range(len(route) + 1):
                    possible_route = route[:insert_index] + [node] + route[insert_index:]
                    possible_time = self.route_time(start, possible_route)

                    if possible_time > max_time:
                        continue

                    if not self.will_need_water(node, possible_time, threshold):
                        continue

                    if best_time is None or possible_time < best_time:
                        best_route = possible_route
                        best_node = node
                        best_time = possible_time

            if best_route is not None:
                route = best_route
                candidates.remove(best_node)
                added_something = True

        return route

    def will_need_water(self, node, seconds_from_now, threshold):
        predicted_moisture = node.moisture - (node.moisture_drop_per_second * seconds_from_now)
        return predicted_moisture <= threshold

    def route_time(self, start, route):
        speed = float(self.get_parameter('average_drive_speed').value)
        irrigation_time = float(self.get_parameter('irrigation_time_per_node').value)

        total_time = 0.0
        current = start

        for node in route:
            total_time += self.distance(current, (node.x, node.y)) / speed
            total_time += irrigation_time
            current = (node.x, node.y)

        return total_time

    def publish_route(self, route):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for node in route:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = Point(x=node.x, y=node.y, z=0.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.poses.append(pose)

        self.route_publisher.publish(msg)

    def distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = IrrigationRoutePlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
