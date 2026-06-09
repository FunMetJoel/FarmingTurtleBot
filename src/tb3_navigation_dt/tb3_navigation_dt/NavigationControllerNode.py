import math

import rclpy
from custom_interfaces.action import SimpleNavigation
from geometry_msgs.msg import Transform, TransformStamped, Twist, TwistStamped, Vector3, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage # pyright: ignore[reportMissingImports]
from tf2_ros import TransformException # pyright: ignore[reportMissingImports]
from tf2_ros.buffer import Buffer # pyright: ignore[reportMissingImports]
from tf2_ros.transform_listener import TransformListener # pyright: ignore[reportMissingImports]


class NavigationControllerNode(Node):

    def __init__(self):
        super().__init__('navigation_controller')

        self._goal_position = None

        self._manhattanNavigationActionServer = ActionServer(
            self,
            SimpleNavigation,
            'manhattan_navigation',
            self._handle_goToPose,
        )
        self._cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_raw', 10)

        self.get_logger().info('NavigationControllerNode initialized and action server started.')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info('TF listener initialized, waiting for transforms to become available...')


        self.nav:BasicNavigator = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active and ready to receive goals.')

        self._isNavigating = False

    def manhattan_navigation(self, goal_handle):
        movingUp = True
        self.get_logger().info('Received goal request: Move to (%.2f, %.2f)' % (goal_handle.request.goal.x, goal_handle.request.goal.y))
        self._goal_position = (goal_handle.request.goal.x, goal_handle.request.goal.y)

        feedback_msg = SimpleNavigation.Feedback()

        # --- Wait for the map -> base_link transform to exist ---
        self.get_logger().info('Waiting for map to base_link transform...')
        while rclpy.ok():
            if self._tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().info('Transform found! Starting navigation...')
                break
            self.get_logger().warn('Still waiting for "map" frame...')
        # ---------------------------------------------------------

        # Create a rate object so your loop runs at a sane speed (e.g., 10 Hz)
        rate = self.create_rate(10)

        while rclpy.ok():
            self.get_logger().info('Checking current position relative to goal...')
            try:
                # Use Time(seconds=0) to grab the latest available transform
                now = rclpy.time.Time(seconds=0)
                transform = self._tf_buffer.lookup_transform('map', 'base_link', now)
                
                current_x = transform.transform.translation.x
                current_y = transform.transform.translation.y
                current_angle = transform.transform.rotation.z  # Assuming 2D navigation, we only care about the z rotation
                self.get_logger().info('Current position: (%.2f, %.2f)' % (current_x, current_y))
                self.get_logger().info('Current angle: %.2f' % current_angle)

                # Calculate distance
                distance = abs(self._goal_position[0] - current_x) + abs(self._goal_position[1] - current_y)
                
                feedback_msg.distance_to_goal = distance 
                goal_handle.publish_feedback(feedback_msg)
                
                if distance < 0.1:
                    self.get_logger().info('Goal reached!')
                    break
                    
            except TransformException as e:
                self.get_logger().warn('Could not get transform: %s' % str(e))

            twistStamped:TwistStamped = TwistStamped()
            twistStamped.header.stamp = self.get_clock().now().to_msg()
            twistStamped.header.frame_id = "base_link"

            if movingUp:
                self.get_logger().info('Moving up...')

                # Check the robot's rotation
                if abs(transform.transform.rotation.z) > 0.1:  # If the robot is rotated more than 0.1 radians
                    self.get_logger().info('Rotating to align with Y-axis...')
                    twistStamped.twist.angular.z = 0.5  # Rotate at 0.5 rad/s
                    self._cmd_vel_publisher.publish(twistStamped)
                else:
                    self.get_logger().info('Moving forward along Y-axis...')
                    twistStamped.twist.linear.x = 0.2  # Move forward at 0.2 m/s
                    self._cmd_vel_publisher.publish(twistStamped)
            
            rate.sleep()

        goal_handle.succeed()
        result = SimpleNavigation.Result()
        result.error_code = 0
        return result        
        
    @property
    def position(self):
        try:
            now = rclpy.time.Time()
            transform = self._tf_buffer.lookup_transform('map', 'base_link', now)
            return (transform.transform.translation.x, transform.transform.translation.y)
        except TransformException as e:
            self.get_logger().warn('Could not get current position: %s' % str(e))
            return None
    
    @property
    def angle(self):
        try:
            now = rclpy.time.Time()
            transform = self._tf_buffer.lookup_transform('map', 'base_link', now)
            return transform.transform.rotation.z  # Assuming 2D navigation, we only care about the z rotation
        except TransformException as e:
            self.get_logger().warn('Could not get current angle: %s' % str(e))
            return None
    
    @property
    def pos(self):
        try:
            now = rclpy.time.Time()
            transform = self._tf_buffer.lookup_transform('map', 'base_link', now)
            return (transform.transform.translation.x, transform.transform.translation.y, transform.transform.rotation.z)
        except TransformException as e:
            self.get_logger().warn('Could not get current position and angle: %s' % str(e))
            return None
    
    def goToPos(self, x:float, y:float, angle:float = 0.0):
        self.get_logger().info('Navigating to position: (%.2f, %.2f)' % (x, y))
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(angle / 2.0)  # Convert angle to quaternion (assuming roll and pitch are 0)
        goal.pose.orientation.w = math.cos(angle / 2.0)
        self.nav.goToPose(goal)

    def _handle_goToPose(self, goal_handle):
        x = goal_handle.request.goal.x
        y = goal_handle.request.goal.y
        angle = goal_handle.request.goal.theta
        self.goToPos(x, y, angle)

        rate = self.create_rate(10)

        while rclpy.ok():
            if self.nav.isTaskComplete():
                # result = SimpleNavigation.Result()
                # if self.nav.getResult() == TaskResult.SUCCEEDED:
                #     self.get_logger().info('Successfully reached the goal!')
                #     result.error_code = 0
                # else:
                #     self.get_logger().warn('Failed to reach the goal.')
                #     result.error_code = 1
                # goal_handle.succeed()
                # return result
                break
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()