from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Int32

from tb3_state_dt.enums import SystemMode


class ModeGuard:
    def __init__(self, node, default_mode=SystemMode.IDLE):
        self.node = node
        self.mode = default_mode

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscription = node.create_subscription(
            Int32,
            "/mode",
            self._mode_callback,
            qos,
        )

    def _mode_callback(self, msg):
        try:
            self.mode = SystemMode(msg.data)
            self.node.get_logger().info(f"System mode is now {self.mode.name}")
        except ValueError:
            self.node.get_logger().warning(f"Ignoring invalid /mode value: {msg.data}")

    def is_simulating(self):
        return self.mode == SystemMode.SIMULATING

    def allows_real_effects(self):
        return not self.is_simulating()