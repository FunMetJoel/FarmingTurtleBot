import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tb3_field_dt.RealField import RealField
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FieldHumiditySensorNode(Node):

    def __init__(self):
        super().__init__('FieldHumiditySensor')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Float64, '/humidity', qos_profile)

        self.timer_ = self.create_timer(1, self.publish_humidity)

        self.field = RealField()
        self.msg = Float64()
        self.randomizer = random.Random()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            "RandomHumiditySensor node started, publishing to /humidity(Float64)"
        )

    def get_location(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                now
            )
            return trans.transform
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform find position {ex}'
            )

    def publish_humidity(self):
        location = self.get_location()
        if location is not None:
            self.msg.data = self.field.get_humidity_at(location.translation.x, location.translation.y)
        else:
            self.msg.data = self.randomizer.random() * 10
            self.get_logger().info(
                f'Could not get location, publishing random humidity value: {self.msg.data}'
            )
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = FieldHumiditySensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
