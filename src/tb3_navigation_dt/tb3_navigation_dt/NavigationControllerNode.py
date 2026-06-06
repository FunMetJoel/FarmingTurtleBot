import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, TransformStamped, Transform
from tf2_msgs.msg import TFMessage # pyright: ignore[reportMissingImports]
from tf2_ros import TransformException # pyright: ignore[reportMissingImports]
from tf2_ros.buffer import Buffer # pyright: ignore[reportMissingImports]
from tf2_ros.transform_listener import TransformListener # pyright: ignore[reportMissingImports]
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from rclpy.action import ActionServer
from custom_interfaces.action import SimpleNavigation
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class NavigationControllerNode(Node):

    def __init__(self):
        super().__init__('navigation_controller')

        self.goal_position = None

        self.manhattanNavigationActionServer = ActionServer(
            self,
            SimpleNavigation,
            'manhattan_navigation',
            self.manhattan_navigation
        )
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_raw', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def manhattan_navigation(self, goal_handle):
        movingUp = True
        self.get_logger().info('Received goal request: Move to (%.2f, %.2f)' % (goal_handle.request.goal.x, goal_handle.request.goal.y))
        self.goal_position = (goal_handle.request.goal.x, goal_handle.request.goal.y)

        feedback_msg = SimpleNavigation.Feedback()

        # --- Wait for the map -> base_link transform to exist ---
        self.get_logger().info('Waiting for map to base_link transform...')
        while rclpy.ok():
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
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
                transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
                
                current_x = transform.transform.translation.x
                current_y = transform.transform.translation.y
                current_angle = transform.transform.rotation.z  # Assuming 2D navigation, we only care about the z rotation
                self.get_logger().info('Current position: (%.2f, %.2f)' % (current_x, current_y))
                self.get_logger().info('Current angle: %.2f' % current_angle)

                # Calculate distance
                distance = abs(self.goal_position[0] - current_x) + abs(self.goal_position[1] - current_y)
                
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
                    self.cmd_vel_publisher.publish(twistStamped)
                else:
                    self.get_logger().info('Moving forward along Y-axis...')
                    twistStamped.twist.linear.x = 0.2  # Move forward at 0.2 m/s
                    self.cmd_vel_publisher.publish(twistStamped)
            
            rate.sleep()

        goal_handle.succeed()
        result = SimpleNavigation.Result()
        result.error_code = 0
        return result

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