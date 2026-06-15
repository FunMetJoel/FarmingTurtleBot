import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import MultiThreadedExecutor
from tb3_navigation_dt.simpleNavigator import SimpleNavigator
from tf2_ros import Buffer, TransformListener



class Nav2IrrigationRouteFollowerNode(SimpleNavigator):

    def __init__(self):
        super().__init__('Nav2IrrigationRouteFollower')

        self.create_subscription(Path, '/irrigation/route', self.route_callback, 10)
        self.accepted_route = False
        self.route = []
        self.current_goal_index = 0
        self.pause_until = None
        self.route_finished = False

    def route_callback(self, msg):
        self.get_logger().info("Route callback")
        if len(msg.poses) == 0:
            return

        if self.accepted_route:
            return

        self.route = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_goal_index = 0
        self.pause_until = None
        self.route_finished = False
        self.accepted_route = True
        self.returning_home = False

        points = []
        for index, point in enumerate(self.route):
            points.append(f'{index + 1}=({point[0]:.2f}, {point[1]:.2f})')

        self.get_logger().info(f'Received one irrigation route with {len(self.route)} waypoints')
        self.get_logger().info('Waypoints: ' + ', '.join(points))

        self.start()

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Point {self.route[self.current_goal_index]} reached successfully!')
            self.current_goal_index = (self.current_goal_index + 1) % len(self.route)
            self.send_goal(*self.route[self.current_goal_index])
        else:
            self.get_logger().info(f'Failed to reach point {self.route[self.current_goal_index]} with status code: {status}')

    def start(self):
        self.send_goal(*self.route[self.current_goal_index])


def main():
    rclpy.init()
    node = Nav2IrrigationRouteFollowerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()