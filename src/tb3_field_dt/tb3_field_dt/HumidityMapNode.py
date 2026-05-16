import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from random import Random
import numpy as np

class HumidityMapNode(Node):

    def __init__(self):
        super().__init__('HumidityMapper')

        self.publisher_ = self.create_publisher(OccupancyGrid, "/humidityMap", 10)

        self.timer_ = self.create_timer(1, self.publish_humidity_map)

        self.get_logger().info(
            "HumidityMap node started, publishing to /humidityMap(OccupancyGrid)"
        )

    def publish_humidity_map(self):
        width = Random().randint(1, 100)
        height = Random().randint(1, 100)
        data = OccupancyGrid()
        data.header.frame_id = 'map'
        data.header.stamp = self.get_clock().now().to_msg()

        data.info.resolution = 0.1
        data.info.width = width#100
        data.info.height = height#100

        origin = Pose()
        origin.position = Point(x=0.0, y=0.0, z=0.0)
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        data.info.origin = origin

        mapData = np.random.randint(-1, 100, (data.info.height, data.info.width))#np.full((data.info.height, data.info.width), -1, dtype=np.int8)
        data.data = mapData.flatten().tolist()

        self.publisher_.publish(data)
        self.get_logger().info(
            "Published"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumidityMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown90


if __name__ == '__main__':
    main()

