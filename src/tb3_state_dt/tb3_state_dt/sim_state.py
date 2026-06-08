import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .enums import RobotState

class SimState(Node):
    
    IGNORE_SYNC_DURATION_NS = 500_000_000
    
    def __init__(self):
        super().__init__('sim_state')
        
        # Start by assuming that the robot is idle.
        self.state = RobotState.IDLE
        self.last_change_time = self.get_clock().now()
        self.get_logger().info(f'started, assuming {self.state.name}')
        
        # Subscribe to the real robot state topic.
        self.subscription_ = self.create_subscription(
            String,
            '/rob_to_sim_state',
            self.sync_state,
            10
        )
        
        # Subscribe to the command topic to manually set the state.
        self.cmd_subscription = self.create_subscription(
            String,
            '/cmd_sim_state',
            self.cmd_state,
            10
        )
        
        # Create a topic that we publish to for syncing state.
        self.state_publisher = self.create_publisher(
            String,
            '/sim_to_rob_state',
            10
        )
        
        # Publish the state periodically.
        self.timer1 = self.create_timer(0.1, self.publish_state)
        
    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_publisher.publish(msg)
        
    def cmd_state(self, msg: String):
        try:
            new_state = RobotState(msg.data)
            if self.state != new_state:
                self.get_logger().info(f'commanded state, now {new_state.name}')
                self.state = new_state
                self.last_change_time = self.get_clock().now()
        except ValueError:
            self.get_logger().error(f'received invalid command string: {msg.data}')
        
    def sync_state(self, msg: String):
        # Ignore syncs if we just changed our state (e.g. within the last 0.5 seconds),
        # so we don't accidentally revert to an old state sent by the other side before
        # it received our new state.
        if (self.get_clock().now() - self.last_change_time).nanoseconds < self.IGNORE_SYNC_DURATION_NS:
            return
            
        try:
            rob_state = RobotState(msg.data)
            
            if self.state != rob_state:
                self.get_logger().info(f'syncing state from robot, now {rob_state.name}')
                self.state = rob_state
                self.last_change_time = self.get_clock().now()
                
        except ValueError:
            self.get_logger().error(f'received invalid state string: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
