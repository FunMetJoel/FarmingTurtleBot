import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64

class SimBatteryLevel(Node):

    def __init__(self):
        super().__init__('battery_level_controller')

        # At the start assume that the battery is empty.
        self.battery_level = 0.0
        self.get_logger().info('started, assuming 0.0')

        # Subscribe to the battery state topic. This will
        # have the data from the real battery.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription_ = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.sync_battery_level,
            sensor_qos
        )

        # Create a topic that we publish to if the battery level is updated
        # based on the battery level, and a topic we publish the
        # errors when syncing with the battery.
        self.battery_level_publisher = self.create_publisher(
            Float64,
            '/sim_battery_level',
            10
        )
        self.battery_error_publisher = self.create_publisher(
            Float64,
            '/sim_battery_level_error',
            10
        )

    def sync_battery_level(self, msg: BatteryState):
        battery_level_error = abs(self.battery_level - msg.percentage)

        error_msg = Float64()
        error_msg.data = battery_level_error
        self.battery_error_publisher.publish(error_msg)
        
        self.battery_level = msg.data
        self.battery_level_publisher.publish(msg)
        self.get_logger().info(f'synced with error {battery_level_error:.4f}, now at {self.battery_level:.4f}')

    # TODO: Add actual simulation of the battery depleting.
    # TODO: Add a service for validating if a set of actions takes too much battery.
    # TODO: Some kind of battery low alert perhaps?


def main(args=None):
    rclpy.init(args=args)
    node = SimBatteryLevel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()