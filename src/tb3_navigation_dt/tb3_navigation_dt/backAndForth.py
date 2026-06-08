import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tb3_navigation_dt.simpleNavigator import SimpleNavigator
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor

class BackAndForth(SimpleNavigator):
    def __init__(self):
        super().__init__('back_and_forth')
        self.points = [(1.0, 0.0), (0.0, 0.0), (0.0, -1.0), (0.0, 0.0)]
        self.current_point_index = 0
        self.get_logger().info('BackAndForth node initialized.')

    def start(self):
        self.send_goal(*self.points[self.current_point_index])

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Point {self.points[self.current_point_index]} reached successfully!')
            self.current_point_index = (self.current_point_index + 1) % len(self.points)
            self.send_goal(*self.points[self.current_point_index])
        else:
            self.get_logger().info(f'Failed to reach point {self.points[self.current_point_index]} with status code: {status}')

def main():
    rclpy.init()
    node = BackAndForth()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()