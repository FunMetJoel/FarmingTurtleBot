import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class RandomHumiditySensorNode(Node):

    def __init__(self):
        super().__init__('RandomHumiditySensor')

        self.publisher_ = self.create_publisher(Float64, '/humidity', 10)

        self.timer_ = self.create_timer(0.1, self.publish_humidity)

        self.get_logger().info(
            "RandomHumiditySensor node started, publishing to /humidity(Float64)"
        )

    def publish_humidity(self):
        data = Float64()
        data.data = random.Random().random()
        self.publisher_.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = RandomHumiditySensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown90


if __name__ == '__main__':
    main()
