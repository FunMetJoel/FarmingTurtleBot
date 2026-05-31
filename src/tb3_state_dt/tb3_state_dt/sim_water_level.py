import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from custom_interfaces.srv import ValidateWaterUsage

class SimWaterLevel(Node):

    def __init__(self):
        super().__init__('sim_water_level')

        # At the start assume that our water tank is empty.
        self.water_level = 0.0
        self.get_logger().info(f'started, now at {self.water_level:.4f}')
        
        # Subscribe to the topic that is publishing the robots water level, and
        # the fill and use commands to determine the resulting error.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Float64,
            '/rob_water_level',
            self.sync_water_level,
            sensor_qos
        )
        
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.fill_subscription = self.create_subscription(
            Float64,
            '/cmd_water_fill',
            self.simulate_filling,
            cmd_qos
        )
        self.use_subscription = self.create_subscription(
            Float64,
            '/cmd_water_use',
            self.simulate_irrigating,
            cmd_qos
        )
        
        # Create a topic that we publish to if the water level is updated
        # based on the real water level, and a topic we publish the
        # errors when syncing with the water tank.
        self.water_level_publisher = self.create_publisher(
            Float64,
            '/sim_water_level',
            10
        )
        self.water_error_publisher = self.create_publisher(
            Float64,
            '/sim_water_level_error',
            10
        )
        
        self.validate_water_usage_service = self.create_service(
            ValidateWaterUsage,
            '/validate_water_usage',
            self.validate_water_usage_callback
        )

    # Every time a message is received from the robots water tank, check
    # the difference between the robot and the simulation. Report the
    # error of the simulation, then update the simulations value and report that as well.
    def sync_water_level(self, msg: Float64):
        water_level_error = abs(self.water_level - msg.data)

        error_msg = Float64()
        error_msg.data = water_level_error
        self.water_error_publisher.publish(error_msg)
        
        self.water_level = msg.data
        self.water_level_publisher.publish(msg)
        self.get_logger().info(f'synced with error {water_level_error:.4f}, now at {self.water_level:.4f}')

    # Change the simulated water level assuming the water tank is ideal.
    def simulate_filling(self, msg: Float64):
        goal = msg.data
        
        # Check this compared to the current water level.
        if (goal < self.water_level):
            self.get_logger().warning(f"target fill lower than current level")
        self.water_level = max(goal, self.water_level)

        # Clamp the capacity of the water tank.
        if (self.water_level > 1.0):
            self.get_logger().warning(f"over filled tank to {self.water_level:.4f}")
            self.water_level = 1.0

        self.get_logger().info(f'fill, now at {self.water_level:.4f}')

    def simulate_irrigating(self, msg: Float64):
        amount = msg.data
        if (amount < 0.0):
            self.get_logger().warning(f"requested negative amount for irrigation, ignoring")
            return

        # Adjust the water level and check if it is negative.
        self.water_level = self.water_level - amount
        if (self.water_level < 0.0):
            self.get_logger().warning(f"dispensed more water than available")
            self.water_level = 0.0

        self.get_logger().info(f'use of {amount:.4f}, now at {self.water_level:.4f}')

    def validate_water_usage_callback(self, request, response):
        # Validate that there is enough water to serve the requested usage
        if request.amount < 0.0:
            self.get_logger().warning("validation failed, negative amount requested")
            response.usage_possible = False
        else:
            response.usage_possible = request.amount <= self.water_level
            self.get_logger().info(f'validation {"success" if response.usage_possible else "failed"}, requested {request.amount:.4f} while {self.water_level:.4f} available')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimWaterLevel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



