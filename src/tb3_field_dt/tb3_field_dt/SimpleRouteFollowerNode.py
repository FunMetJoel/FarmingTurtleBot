import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class SimpleRouteFollowerNode(Node):

    def __init__(self):
        super().__init__('SimpleRouteFollower')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('turn_first_angle', 0.35)
        self.declare_parameter('irrigation_pause_seconds', 1.0)
        self.declare_parameter('water_per_node', 0.10)
        self.declare_parameter('watered_moisture', 1.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('refill_level', 1.0)

        self.route = []
        self.current_goal_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False
        self.water_level = None
        self.pause_until = None
        self.route_finished = False
        self.accepted_route = False
        self.returning_home = False
        self.last_announced_goal = -1
        self.stop_sent = False

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.create_subscription(PoseArray, '/irrigation/route', self.route_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            Float64,
            '/rob_water_level',
            self.water_level_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.water_use_publisher = self.create_publisher(
            Float64,
            '/cmd_water_use',
            10
        )
        self.water_fill_publisher = self.create_publisher(
            Float64,
            '/cmd_water_fill',
            10
        )
        self.field_water_publisher = self.create_publisher(
            Float64,
            '/water',
            10
        )
        self.irrigating_publisher = self.create_publisher(
            Bool,
            '/irrigating',
            10
        )
        self.timer = self.create_timer(0.1, self.drive_step)

        self.get_logger().info('Simple route follower started')

    def route_callback(self, msg):
        if len(msg.poses) == 0:
            return

        if self.accepted_route:
            return

        self.route = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.current_goal_index = 0
        self.pause_until = None
        self.route_finished = False
        self.accepted_route = True
        self.returning_home = False

        points = []
        for index, point in enumerate(self.route):
            points.append(f'{index + 1}=({point[0]:.2f}, {point[1]:.2f})')

        self.get_logger().info(f'Received one irrigation route with {len(self.route)} waypoints')
        self.get_logger().info('Waypoints: ' + ', '.join(points))

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.has_odom = True

    def water_level_callback(self, msg):
        self.water_level = msg.data

    def drive_step(self):
        if (
            not self.has_odom
            or self.water_level is None
            or not self.accepted_route
            or self.route_finished
        ):
            return

        if self.pause_until is not None:
            if self.get_clock().now().nanoseconds < self.pause_until:
                self.stop_robot()
                return
            self.pause_until = None
            self.publish_irrigating(False)

        if not self.returning_home:
            water_per_node = float(
                self.get_parameter('water_per_node').value
            )
            if self.water_level < water_per_node:
                self.start_return_home('Water level is too low')
            elif self.current_goal_index >= len(self.route):
                self.start_return_home('Finished irrigation route')

        if self.returning_home:
            goal_x = float(self.get_parameter('home_x').value)
            goal_y = float(self.get_parameter('home_y').value)
        else:
            goal_x, goal_y = self.route[self.current_goal_index]

        should_announce_goal = (
            not self.returning_home
            and self.last_announced_goal != self.current_goal_index
        )
        if should_announce_goal:
            self.last_announced_goal = self.current_goal_index
            self.get_logger().info(
                f'Driving to waypoint {self.current_goal_index + 1}/{len(self.route)} '
                f'at ({goal_x:.2f}, {goal_y:.2f})'
            )

        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        tolerance = float(self.get_parameter('goal_tolerance').value)
        if distance < tolerance:
            if self.returning_home:
                self.refill_and_wait()
            else:
                self.irrigate_current_waypoint()
            return

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.robot_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        turn_first_angle = float(self.get_parameter('turn_first_angle').value)

        if abs(angle_error) > turn_first_angle:
            angular_speed = float(
                self.get_parameter('angular_speed').value
            )
            cmd.twist.angular.z = self.sign(angle_error) * angular_speed
        else:
            cmd.twist.linear.x = float(self.get_parameter('linear_speed').value)
            cmd.twist.angular.z = angle_error

        self.stop_sent = False
        self.cmd_publisher.publish(cmd)

    def irrigate_current_waypoint(self):
        waypoint_number = self.current_goal_index + 1
        self.get_logger().info(
            f'Reached waypoint {waypoint_number}/{len(self.route)}. '
            f'Irrigating briefly.'
        )

        water_per_node = float(self.get_parameter('water_per_node').value)
        self.water_use_publisher.publish(Float64(data=water_per_node))
        watered_moisture = float(
            self.get_parameter('watered_moisture').value
        )
        self.field_water_publisher.publish(Float64(data=watered_moisture))
        self.water_level = max(0.0, self.water_level - water_per_node)
        self.publish_irrigating(True)

        self.current_goal_index += 1
        pause_seconds = float(
            self.get_parameter('irrigation_pause_seconds').value
        )
        self.pause_until = (
            self.get_clock().now().nanoseconds
            + int(pause_seconds * 1000000000)
        )
        self.stop_robot()

    def start_return_home(self, reason):
        self.returning_home = True
        self.publish_irrigating(False)
        self.get_logger().info(f'{reason}. Returning home to refill.')

    def refill_and_wait(self):
        refill_level = float(self.get_parameter('refill_level').value)
        self.water_fill_publisher.publish(Float64(data=refill_level))
        self.water_level = refill_level
        self.publish_irrigating(False)
        self.route_finished = True
        self.stop_robot()
        self.timer.cancel()
        self.get_logger().info('Reached home, refilled water, and waiting.')

    def publish_irrigating(self, irrigating):
        self.irrigating_publisher.publish(Bool(data=irrigating))

    def stop_robot(self):
        if self.stop_sent:
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        self.cmd_publisher.publish(cmd)
        self.stop_sent = True

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def sign(self, value):
        if value >= 0.0:
            return 1.0
        return -1.0


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRouteFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
