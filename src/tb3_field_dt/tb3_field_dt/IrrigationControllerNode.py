import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class IrrigationControllerNode(Node):
        
    def __init__(self):
        super().__init__('irrigation_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to the real robot state topic.
        self.create_subscription(
            Bool,
            '/irrigate',
            self.on_irrigate,
            10
        )

        self.create_subscription(
            Float64,
            '/humidity',
            self.collect_humidity_data,
            qos_profile
        )
        
        # Create a topic that we publish to for syncing state.
        self.water_use_publisher = self.create_publisher(
            Float64,
            '/cmd_water_use',
            10
        )

        self.humidity = 0.0
        self.target_humidity = 1.0

    def on_irrigate(self, msg):
        if msg.data is True:
            waterNeeded = self.target_humidity - self.humidity
            new_msg = Float64()
            new_msg.data = waterNeeded / 20.0
            self.get_logger().info(f"Water needed: {waterNeeded}")
            self.water_use_publisher.publish(new_msg)

    def collect_humidity_data(self, msg):
        self.humidity = msg.data
        self.get_logger().info(f"Humidity: {self.humidity}")

def main(args=None):
    rclpy.init(args=args)
    node = IrrigationControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
