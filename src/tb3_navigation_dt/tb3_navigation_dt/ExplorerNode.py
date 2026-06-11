import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs_py import point_cloud2
from tb3_navigation_dt.simpleNavigator import SimpleNavigator
from tb3_navigation_dt.mapping_planner import generate_mapping_waypoints
from tb3_navigation_dt.zigzag_planner import generate_zigzag_waypoints
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, Point, Pose
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Header
import math
import matplotlib.pyplot as plt



class Explorer(SimpleNavigator):
    def __init__(self):
        super().__init__('explorer')
        self.points = [(0.0, 0.0)]
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
            PointCloud2,
            '/pointsToGoTo',
            10
        )

        self.lowResMapPublisher = self.create_publisher(
            Path,
            '/lowResMap',
            10
        )

        self.visitedWaypoints = set([])
        self.currGoal = None

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().debug('Received new map data, generating waypoints...')
        self.map_msg = msg
        if self.humidity_map_msg is not None:
            self.points, map = generate_mapping_waypoints(self.map_msg, self.humidity_map_msg)
            # # plt.figure(figsize=(10, 10))
            # plt.imshow(map, cmap='gray', origin='lower')
            # # plt.colorbar(label='Reachable')
            # # plt.title('LOWRES')
            # # plt.xlabel('Column')
            # # plt.ylabel('Row')
            # plt.savefig('REAC.png', dpi=150, bbox_inches='tight')
            # plt.close()
            if not self.points:
                self.get_logger().warn('No free space found in the map to generate waypoints.')
            else:
                self.get_logger().info(f'Generated {len(self.points)} waypoints.')
            
            header = Header()
            header.frame_id = 'map'
            points = [(p[0], p[1], 0.0) for p in self.points]
            publish_msg = point_cloud2.create_cloud_xyz32(header, points)
            
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

    def humidity_map_callback(self, msg: OccupancyGrid):
        self.humidity_map_msg = msg
        
    def start(self):
        self.send_goal(0, 0)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Point {self.currGoal} reached successfully!')
            self.visitedWaypoints.add(self.currGoal)
        else:
            self.get_logger().warn(f'Failed to reach point {self.currGoal} with status code: {status}')

        goal = self.getNextGoal()
        if goal is None:
            pass
        else:
            self.currGoal = goal
            self.send_goal(*goal)

    def getNextGoal(self):
        pos = self.getRobotPos()
        leastDistance = math.inf
        currentClosest = (None)
        for point in self.points:
            if point in self.visitedWaypoints:
                continue
            distance = (2 * (pos[0] - point[0]) ** 2) + ((pos[1] - point[1]) ** 2)
            if distance < leastDistance:
                currentClosest = point
                leastDistance = distance
        return currentClosest


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