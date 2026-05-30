import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RandomHumiditySensorNode(Node):

    def __init__(self):
        super().__init__('RandomHumiditySensor')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Float64, '/humidity', qos_profile)

        self.timer_ = self.create_timer(1, self.publish_humidity)

        self.randomizer = random.Random()
        self.msg = Float64()

        self.get_logger().info(
            "RandomHumiditySensor node started, publishing to /humidity(Float64)"
        )

    def publish_humidity(self):
        self.msg.data = self.randomizer.random() * 10
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomHumiditySensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
