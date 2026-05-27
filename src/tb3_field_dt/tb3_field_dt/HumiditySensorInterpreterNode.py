import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, TransformStamped, Transform
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class HumiditySensorInterpreterNode(Node):

    def __init__(self):
        super().__init__('HumiditySensorInterpreter')

        self.publisher_ = self.create_publisher(Vector3, '/locatedHumidityData', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription_humidity = self.create_subscription(
            Float64,
            '/humidity',
            self.publish_humidity,
            qos_profile
        )
        
        self.currentLocation: Transform = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.get_robot_position)

        self.get_logger().info(
            "RandomHumiditySensor node started, publishing to /locatedHumidityData(Vector3)"
        )

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


    def extract_location(self, msg:TFMessage):
        for transformStamped in msg.transforms:
            transformStamped:TransformStamped = transformStamped
            if (transformStamped.header.frame_id == 'odom') and (transformStamped.child_frame_id == 'base_footprint'):
                self.odomToRobot = transformStamped.transform
            if (transformStamped.header.frame_id == 'map') and (transformStamped.child_frame_id == 'odom'):
                self.mapToOdom = transformStamped.transform

        if self.mapToOdom == None or self.odomToRobot == None:
            self.get_logger().info(
                f"Did not have both datapoints, {self.mapToOdom == None}, {self.odomToRobot == None}"
            )
            return

        location: Transform = Transform()
        location.translation.x = self.odomToRobot.translation.x + self.mapToOdom.translation.x
        location.translation.y = self.odomToRobot.translation.y + self.mapToOdom.translation.y
        location.translation.z = 0.0

        location.rotation.x = self.odomToRobot.rotation.x + self.mapToOdom.rotation.x
        location.rotation.y = self.odomToRobot.rotation.y + self.mapToOdom.rotation.y
        location.rotation.z = self.odomToRobot.rotation.z + self.mapToOdom.rotation.z
        location.rotation.w = self.odomToRobot.rotation.w + self.mapToOdom.rotation.w

        self.currentLocation = location


    def publish_humidity(self, msg:Float64):
        if self.currentLocation == None:
            self.get_logger().info(
                "No location info found yet"
            )
            return
        data = Vector3()
        data.x = self.currentLocation.translation.x
        data.y = self.currentLocation.translation.y
        data.z = msg.data
        self.publisher_.publish(data)
        self.get_logger().info(
            f"Published {data.x}, {data.y}: {data.z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumiditySensorInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
