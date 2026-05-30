import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, TransformStamped, Transform
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math


class RowMotionNode(Node):

    def __init__(self):
        super().__init__('row_motion')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_raw', 10)

        self.movingFowardState = True
        self.startup = True
        self.currentLocation: Transform = None


    def get_robot_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                now
            )

            self.currentLocation: Transform = trans.transform
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform find position'
            )
        
    def send_initial_command(self):
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
                twistStamped.twist.linear.x = 0.0
                twistStamped.twist.angular.z = 0.3 if self.getCloserSide(msg) else -0.3
                self.publisher.publish(twistStamped)
            if (not self.movingFowardState) and bigger_range_front_distance > 1:
                self.get_logger().info(f'Path clear. Distance: {front_distance:.2f}m')
                self.movingFowardState = True
                twistStamped.twist.linear.x = 0.2
                twistStamped.twist.angular.z = 0.0
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
    row_motion = RowMotionNode()
    rclpy.spin(row_motion)
    row_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()