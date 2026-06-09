import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class SimpleNavigator(Node):
    def __init__(self, node_name='simple_navigator'):
        super().__init__(node_name)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: ({x}, {y})')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected (ugh)')
            return

        self.get_logger().info('Goal accepted (yay)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Current robot pose: ({feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y})')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was canceled.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('Goal was aborted by the Nav2 server.')
        else:
            self.get_logger().info(f'Goal failed with status code: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    node.send_goal(0.0, 0.0)  # Example target position

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down navigation node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()