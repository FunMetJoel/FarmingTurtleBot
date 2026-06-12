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
from custom_interfaces.action import MapField
from rclpy.action import ActionServer


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

        self._actionServer = ActionServer(
            self,
            MapField,
            '/map_field',
            self._handle_mapField,
        )

        self.visitedWaypoints = set([])
        self.currGoal = None

    def _handle_mapField(self, goal_handle):
        self.get_logger().info("Started")
        self._start()
        self.goal_handle = goal_handle

        rate = self.create_rate(1)
        while rclpy.ok() and self.goal_handle.is_active:

            if (self.humidity_map_msg is None or self.map_msg is None):
                self.get_logger().info("Waiting on map")

            # TODO: Determine when finished
            
            rate.sleep()

        result = MapField.Result()

        if True: # TODO: determine when finished
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().debug('Received new map data, generating waypoints...')
        self.map_msg = msg
        if self.humidity_map_msg is not None:
            self.points = generate_mapping_waypoints(self.map_msg, self.humidity_map_msg)
            if not self.points:
                self.get_logger().warn('No free space found in the map to generate waypoints.')
            else:
                self.get_logger().debug(f'Generated {len(self.points)} waypoints.')
            
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
        
    def _start(self):
        self.get_logger().warn("STHART>??")
        self.currGoal = self.getNextGoal()
        if self.currGoal is None:
            self.send_goal(0, 0)
        else:
            self.send_goal(*self.currGoal)
        

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Point {self.currGoal} reached successfully!')
            self.visitedWaypoints.add(self.currGoal)
        else:
            self.get_logger().warn(f'Failed to reach point {self.currGoal} with status code: {status}')

        self.get_logger().info(f'Visited {len(self.visitedWaypoints)} waypoints, {len([x for x in self.points if x not in self.visitedWaypoints])} to go.')
        
        goal = self.getNextGoal()
        self.get_logger().info("Calculated Goal")
        if goal is None:
            pass
        else:
            self.currGoal = goal
            self.send_goal(*goal)

    def getNextGoal(self):
        pos = self.getRobotPos()
        leastDistance = math.inf
        currentClosest = None
        for point in self.points:
            if point in self.visitedWaypoints:
                continue
            distance = ((pos[0] - point[0]) ** 2) +  (3 * ((pos[1] - point[1]) ** 2))
            if distance < leastDistance:
                currentClosest = point
                leastDistance = distance
        return currentClosest


def main():
    rclpy.init()
    node = Explorer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()