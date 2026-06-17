import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Int32
from tb3_state_dt.enums import SystemMode


class SetModeNode(Node):
    def __init__(self, mode_value: int):
        super().__init__('set_mode')

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(Int32, '/mode', qos)

        msg = Int32()
        msg.data = mode_value

        self.timer = self.create_timer(0.2, lambda: self.publish_once(msg))

    def publish_once(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info(f'Set mode to {SystemMode(msg.data).name}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run tb3_state_dt set_mode <mode_number>')
        print('Example: ros2 run tb3_state_dt set_mode 3')
        return

    mode_value = int(sys.argv[1])
    SystemMode(mode_value)

    node = SetModeNode(mode_value)
    rclpy.spin(node)