import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tb3_navigation_dt.simpleNavigator import SimpleNavigator
from tb3_navigation_dt.mapping_planner import generate_mapping_waypoints
from tb3_navigation_dt.zigzag_planner import generate_zigzag_waypoints
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, Point, Pose
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
import math


class Explorer(SimpleNavigator):
    def __init__(self):
        super().__init__('explorer')
        self.points = [(0.0, 0.0)]
        self.current_point_index = 0
        self.get_logger().info('Explorer node initialized.')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.humidity_map_subscription = self.create_subscription(
            OccupancyGrid,
            '/humidityMap',
            self.humidity_map_callback,
            10
        )
        self.map_msg = None
        self.humidity_map_msg = None

        self.waypoint_publisher = self.create_publisher(
            Path,
            '/pointsToGoTo',
            10
        )

        self.lowResMapPublisher = self.create_publisher(
            Path,
            '/lowResMap',
            10
        )

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received new map data, generating waypoints...')
        self.map_msg = msg
        if self.humidity_map_msg is not None:
            self.points, map = generate_mapping_waypoints(self.map_msg, self.humidity_map_msg)
            if not self.points:
                self.get_logger().warn('No free space found in the map to generate waypoints.')
            else:
                self.get_logger().info(f'Generated {len(self.points)} waypoints.')
            
            self.get_logger().info(f'Generated waypoints: {self.points}')
            publish_msg = Path()
            publish_msg.header.frame_id = 'map'
            now = self.get_clock().now().to_msg()
            publish_msg.header.stamp = now
            for point in self.points:
                posePoint = PoseStamped()
                posePoint.header.frame_id = 'map'
                posePoint.header.stamp = now
                posePoint.pose.position.x = point[0]
                posePoint.pose.position.y = point[1]
                publish_msg.poses.append(posePoint)
            self.waypoint_publisher.publish(publish_msg)

            data = OccupancyGrid()
            data.header.frame_id = 'map'
            data.header.stamp = self.get_clock().now().to_msg()

            data.info = self.humidity_map_msg.info

            origin = Pose()

            origin.position = Point(
                x=math.floor(float(msg.info.origin.position.x) * (1/float(self.humidity_map_msg.info.resolution))) * float(self.humidity_map_msg.info.resolution), 
                y=math.floor(float(msg.info.origin.position.y) * (1/float(self.humidity_map_msg.info.resolution))) * float(self.humidity_map_msg.info.resolution), 
                z=0.0
            )
            data.info.origin = origin

            data.data = map.flatten().tolist()
            self.lowResMapPublisher.publish(data)
            self.get_logger().info(
                "Published MAP"
            )

    def humidity_map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received new humidity map data, generating waypoints...')
        self.humidity_map_msg = msg
        
    def start(self):
        self.send_goal(*self.points[self.current_point_index])

    def get_result_callback(self, future):
        self.get_logger().info(f'{self.humidity_map_msg is not None} {self.map_msg is not None}')
        result = future.result().result
        status = future.result().status
        
        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Point {self.points[self.current_point_index]} reached successfully!')
        else:
            self.get_logger().info(f'Failed to reach point {self.points[self.current_point_index]} with status code: {status}')

        self.current_point_index = (self.current_point_index + 1) % len(self.points)
        self.send_goal(*self.points[self.current_point_index])

def main():
    rclpy.init()
    node = Explorer()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()