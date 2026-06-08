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
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_raw', 10)

        self.movingFowardState = True
        self.startup = True
        self.direction = 0

        
    def sendInitCmd(self):
        twistStamped:TwistStamped = TwistStamped()
        twistStamped.header.stamp = self.get_clock().now().to_msg()
        twistStamped.header.frame_id = "base_link"
        twistStamped.twist.linear.x = 0.2
        twistStamped.twist.angular.z = 0.0
        self.publisher.publish(twistStamped)
        
    def scan_callback(self, msg):
        if self.startup:
            self.sendInitCmd()
            self.startup = False

        # -15 tot 15, in degrees, front is 0 degrees
        front_distance = min(min(msg.ranges[-30:]), min(msg.ranges[:30]))
        bigger_range_front_distance = min(min(msg.ranges[-45:]), min(msg.ranges[:45]))

        self.get_logger().info(f'Front distance: {front_distance:.2f}m, moving forward: {self.movingFowardState}')
        
        if not math.isnan(front_distance) and front_distance > msg.range_min:
            
            twistStamped:TwistStamped = TwistStamped()
            twistStamped.header.stamp = self.get_clock().now().to_msg()
            twistStamped.header.frame_id = "base_link"


            if self.movingFowardState and front_distance < 0.5:
                self.get_logger().info(f'Wall detected! Distance: {front_distance:.2f}m. Stopping.')
                self.movingFowardState = False
                self.direction = 0.3 if self.getCloserSide(msg) else -0.3

            if (not self.movingFowardState) and bigger_range_front_distance > 1:
                self.get_logger().info(f'Path clear. Distance: {front_distance:.2f}m')
                self.movingFowardState = True


            if self.movingFowardState:
                twistStamped.twist.linear.x = 1.0
                twistStamped.twist.angular.z = 0.0
                self.publisher.publish(twistStamped)
            else:
                twistStamped.twist.linear.x = 0.0
                twistStamped.twist.angular.z = self.direction
                self.publisher.publish(twistStamped)

    def getCloserSide(self, scanMsg):
        """Returns True if right is closer, False if left is closer
        Returns:
            bool: True if right is closer, False if left is closer
        """
        leftDistance = min(scanMsg.ranges[-45:-5])
        rightDistance = min(scanMsg.ranges[5:45])
        return rightDistance > leftDistance

def main(args=None):
    rclpy.init(args=args)
    random_walk = RandomWalkNode()
    rclpy.spin(random_walk)
    random_walk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()