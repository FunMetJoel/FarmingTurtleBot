import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from .DynamicMap import DynamicMap

class HumidityMapNode(Node):

    def __init__(self):
        super().__init__('HumidityMapper')

        self.publisher_ = self.create_publisher(OccupancyGrid, "/humidityMap", 10)
        self.humidityDepreciationRate = 0.001

        self.subscription_ = self.create_subscription(
            Vector3,
            '/locatedHumidityData',
            self.scan_callback,
            10
        )

        self.timer_ = self.create_timer(1, self.publish_humidity_map)
        self.lastTime = self.get_clock().now()

        self.get_logger().info(
            "HumidityMap node started, publishing to /humidityMap(OccupancyGrid)"
        )

        self.map = DynamicMap(0.5, (0, 0), (2, 2))

    def publish_humidity_map(self):
        deltaTime = self.get_clock().now() - self.lastTime
        
        self.map.subtractEverywhere(self.humidityDepreciationRate * deltaTime.nanoseconds / 1e9)

        
        data = OccupancyGrid()
        data.header.frame_id = 'map'
        data.header.stamp = self.get_clock().now().to_msg()

        data.info.resolution = self.map.resolution
        data.info.width = self.map.sizeX
        data.info.height = self.map.sizeY

        origin = Pose()
        origin.position = Point(
            x=float(self.map.originX - self.map.resolution / 2), 
            y=float(self.map.originY - self.map.resolution / 2), 
            z=0.0
        )
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        data.info.origin = origin

        mapData = self.map.toOccupancyGridData(0.0, 1.0)
        data.data = mapData.flatten().tolist()
        self.publisher_.publish(data)
        self.get_logger().info(
            "Published"
        )

        self.lastTime = self.get_clock().now()


    def scan_callback(self, msg:Vector3):
        self.map.setPixelAtLocation(msg.x, msg.y, msg.z)
 
        self.get_logger().info(
            f'Update recieved: {msg.x}, {msg.y}, {msg.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = HumidityMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

