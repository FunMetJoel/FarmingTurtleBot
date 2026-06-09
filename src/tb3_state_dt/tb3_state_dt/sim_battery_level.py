import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, String

class SimBatteryLevel(Node):

    LOW_BATTERY_TRESSHOLD = 0.2
    CRITICAL_BATTERY_TRESSHOLD = 0.05

    def __init__(self):
        super().__init__('battery_level_controller')

        # At the start assume that the battery is about half full.
        self.battery_level = 0.5
        self.get_logger().info(f'started, assuming {self.battery_level}')

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
        self.battery_low_publisher = self.create_publisher(
            String,
            '/sim_battery_low_alert',
            10
        )

        # Check for a low battery level periodically.
        self.timer1 = self.create_timer(1.0, self.battery_low_alert)

    def sync_battery_level(self, msg: BatteryState):
        battery_level_error = abs(self.battery_level - msg.percentage)

        error_msg = Float64()
        error_msg.data = battery_level_error
        self.battery_error_publisher.publish(error_msg)
        
        self.battery_level = msg.percentage

        forward_msg = Float64()
        forward_msg.data = self.battery_level
        self.battery_level_publisher.publish(forward_msg)
        self.get_logger().info(f'synced with error {battery_level_error:.4f}, now at {self.battery_level:.4f}')

    def battery_low_alert(self):

        # If the battery level is above the tresshold, we can skip the iteration.
        if self.battery_level >= self.LOW_BATTERY_TRESSHOLD:
            return

        # Determine if the battery level is low or critical.
        warning_msg = String()
        if self.battery_level >= self.CRITICAL_BATTERY_TRESSHOLD:
            warning_msg.data = "LOW"
        else:
            warning_msg.data = "CRITICAL"

        self.battery_low_publisher.publish(warning_msg)
        self.get_logger().warning(f"battery {warning_msg.data}, now at {self.battery_level:.4f}")

    # Maybe add actual simulation of the battery depleting.
    # Maybe add a service for validating if a set of actions takes too much battery.

def main(args=None):
    rclpy.init(args=args)
    node = SimBatteryLevel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()