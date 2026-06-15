import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from turtlebot3_msgs.srv import Sound


class SoundController(Node):
    
    IGNORE_SYNC_DURATION_NS = 500_000_000
    
    def __init__(self):
        super().__init__('sound_controller')
        
        self.subscription_ = self.create_subscription(
            Bool,
            '/irrigating',
            self.on_irrigation,
            10
        )

        self.sound_client = self.create_client(
            Sound,
            '/sound'
        )
        while not self.sound_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def on_irrigation(self, msg):
        if msg.data is True:
            self.get_logger().info("sending sound")
            req = Sound.Request()
            req.value = 0 # 'ON' sound, we should pick a sound in the lab
            self.sound_client.call_async(req)

        
def main(args=None):
    rclpy.init(args=args)
    node = SoundController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
