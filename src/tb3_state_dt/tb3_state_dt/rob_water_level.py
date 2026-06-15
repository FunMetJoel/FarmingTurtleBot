import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
import random

# This node represents the state of the actual water tank.
# In the real robot, this would be a sensor connected to the
# water tank, that can measure the water level.

class RobWaterLevel(Node):

    def __init__(self):
        super().__init__('rob_water_level')

        self.declare_parameter('initial_water_level', 1.0)

        # The water level is a percentage, represented by a floating
        # point number between zero and one.
        self.water_level = float(
            self.get_parameter('initial_water_level').value
        )
        self.water_level = min(max(self.water_level, 0.0), 1.0)
        self.get_logger().info(f'started, now at {self.water_level:.4f}')

        # The water level is published to a topic periodically.
        self.water_level_publisher = self.create_publisher(Float64, '/rob_water_level', 10)

        # Subscribe to a topic that indicates a request to fill or use water.
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Float64,
            "/cmd_water_fill",
            self.simulate_filling,
            cmd_qos
        )
        self.create_subscription(
            Float64,
            "/cmd_water_use",
            self.simulate_irrigating,
            cmd_qos
        )
        self.water_field_publisher = self.create_publisher(
            Float64,
            "/water",
            cmd_qos
        )

        # Initiate the timers for repeating tasks.
        self.timer1 = self.create_timer(1.0, self.periodic_publish)
        self.timer2 = self.create_timer(10.0, self.periodic_leak)

    # Periodically publish the state of water level.
    def periodic_publish(self):
        msg = Float64()
        msg.data = self.water_level
        self.water_level_publisher.publish(msg)
        self.get_logger().info(f"published, now at {self.water_level:.4f}")

    # Simulate the water tank being filled up to a desired percentage.
    # The real fillup process is not fully accurate, so we are taking
    # a random +-3% error into account here.
    def simulate_filling(self, msg: Float64):
        goal = msg.data

        # Adjust the goal by the possible error and check this compared
        # to the current water level.
        adjusted_goal = goal + random.uniform(-0.03, 0.03)
        if (adjusted_goal < self.water_level):
            self.get_logger().warning(f"target fill lower than current level")
        self.water_level = max(adjusted_goal, self.water_level)

        # Clamp the capacity of the water tank.
        if (self.water_level > 1.0):
            self.get_logger().warning(f"over filled tank to {self.water_level:.4f}")
            self.water_level = 1.0

        self.get_logger().info(f"attempted to fill to {goal:.4f}, now at {self.water_level:.4f}")

    # Simulate the water from the tank being used for irrigation.
    # The parameter indicates the desired percentage of the tank to be
    # released for watering. As this is not perfect, a random error
    # up to 2% is taken into account here.
    def simulate_irrigating(self, msg: Float64):
        amount = msg.data
        if (amount < 0.0):
            self.get_logger().warning(f"requested negative amount for irrigation, ignoring")
            return

        # Adjust the water level and check if it is negative.
        self.water_level = self.water_level - amount - random.uniform(0.0, 0.02)
        if (self.water_level < 0.0):
            self.get_logger().warning(f"dispensed more water than available")
            self.water_level = 0.0

        usedAmount = max(amount, self.water_level)
        new_msg = Float64()
        new_msg.data = usedAmount
        self.water_field_publisher.publish(usedAmount)

        self.get_logger().info(f"used {amount:.4f} for irrigation, now at {self.water_level:.4f}")

    # Water tanks are not perfect, and therefore we are simulating a bit of
    # leaking for the water tank. Water can not leak if the tank is empty.
    def periodic_leak(self):
        self.water_level = max(self.water_level - random.uniform(0.0, 0.005), 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RobWaterLevel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


