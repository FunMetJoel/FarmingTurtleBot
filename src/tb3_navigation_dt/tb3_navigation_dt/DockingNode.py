import numpy as np
from typing import override
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tb3_navigation_dt.simpleNavigator import SimpleNavigator
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

from custom_interfaces.action import DockRobot

class DockingNode(SimpleNavigator):
    def __init__(self):
        super().__init__('Docking')
        self.dockPos = (0.0, 0.0)
        self.get_logger().info('Docking node initialized.')

        self._actionServer = ActionServer(
            self,
            DockRobot,
            'dock_robot',
            self._handle_dock,
        )

        self.goal_handle = None
        self.nav_success = False

    def _start(self):
        self.send_goal(*self.dockPos)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Docking station reached successfully!')
            self.nav_success = True
        else:
            self.get_logger().info(f'Failed to reach Docking station with status code: {status}')
            self.nav_success = False

    def _handle_dock(self, goal_handle):
        self._start()
        self.goal_handle = goal_handle

        rate = self.create_rate(2)
        while rclpy.ok() and self.goal_handle.is_active:
            if self.nav_success: 
                break
            rate.sleep()

        result = DockRobot.Result()

        if self.nav_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result
        

    @override
    def feedback_callback(self, feedback_msg):
        if self.goal_handle is None:
            return
        feedback = feedback_msg.feedback
        new_feedback_msg = DockRobot.Feedback()
        new_feedback_msg.distance_to_goal = feedback.distance_remaining
        self.goal_handle.publish_feedback(new_feedback_msg)




def main():
    rclpy.init()
    node = DockingNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()