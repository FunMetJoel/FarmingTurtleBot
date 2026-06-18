import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Int32

from tb3_state_dt.enums import SystemMode


class SetModeNode(Node):
    def __init__(self, mode_value: int):
        super().__init__('set_mode')

        self.mode = SystemMode(mode_value)
        self.publish_count = 0
        self.max_publish_count = 10

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(Int32, '/mode', qos)

        self.msg = Int32()
        self.msg.data = self.mode.value

        self.timer = self.create_timer(0.2, self.publish_mode)

    def publish_mode(self):
        self.publisher.publish(self.msg)
        self.publish_count += 1

        if self.publish_count == 1:
            self.get_logger().info(f'Setting mode to {self.mode.name}')

        if self.publish_count >= self.max_publish_count:
            self.get_logger().info(f'Set mode to {self.mode.name}')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run tb3_state_dt set_mode <mode_number>')
        print('Example: ros2 run tb3_state_dt set_mode 3')
        rclpy.shutdown()
        return

    try:
        mode_value = int(sys.argv[1])
        SystemMode(mode_value)
    except ValueError:
        print(f'Invalid mode: {sys.argv[1]}')
        print('Valid modes:')
        for mode in SystemMode:
            print(f'  {mode.value}: {mode.name}')
        rclpy.shutdown()
        return

    node = SetModeNode(mode_value)
    rclpy.spin(node)
    node.destroy_node()