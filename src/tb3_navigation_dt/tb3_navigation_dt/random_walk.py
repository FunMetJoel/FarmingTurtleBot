import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Twist
import math

class RandomWalkNode(Node):

    def __init__(self):
        super().__init__('random_walk')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
    def scan_callback(self, msg):
        # -15 tot 15, in degrees, front is 0 degrees
        front_distance = min(min(msg.ranges[-15:]), min(msg.ranges[:15]))
        
        if not math.isnan(front_distance) and front_distance > msg.range_min:
            
            twistStamped:TwistStamped = TwistStamped()
            twistStamped.header.stamp = self.get_clock().now().to_msg()
            twistStamped.header.frame_id = "base_link"
            
            if front_distance < 0.5: # Robot rotates
                self.get_logger().info(f'Wall detected! Distance: {front_distance:.2f}m. Stopping.')
                twistStamped.twist.linear.x = 0.0
                twistStamped.twist.angular.z = 0.3
                self.publisher.publish(twistStamped)
            else: # Robot moves forward
                self.get_logger().info(f'Path clear. Distance: {front_distance:.2f}m')
                twistStamped.twist.linear.x = 0.2
                twistStamped.twist.angular.z = 0.0
                self.publisher.publish(twistStamped)

def main(args=None):
    rclpy.init(args=args)
    random_walk = RandomWalkNode()
    rclpy.spin(random_walk)
    random_walk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()