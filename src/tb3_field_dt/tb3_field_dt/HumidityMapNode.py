import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from random import Random
from .DynamicMap import DynamicMap
import numpy as np

class HumidityMapNode(Node):

    def __init__(self):
        super().__init__('HumidityMapper')

        self.publisher_ = self.create_publisher(OccupancyGrid, "/humidityMap", 10)

        self.timer_ = self.create_timer(1, self.publish_humidity_map)

        self.get_logger().info(
            "HumidityMap node started, publishing to /humidityMap(OccupancyGrid)"
        )

        self.map = DynamicMap(0.1)

    def publish_humidity_map(self):
        
        data = OccupancyGrid()
        data.header.frame_id = 'map'
        data.header.stamp = self.get_clock().now().to_msg()

        data.info.resolution = self.map.resolution
        data.info.width = self.map.size[0]
        data.info.height = self.map.size[1]

        origin = Pose()
        origin.position = Point(x=self.map.origin[0], y=self.map.origin[1], z=0.0)
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        data.info.origin = origin

        mapData = self.map.toOccupancyGridData(0.0, 10.0)
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

