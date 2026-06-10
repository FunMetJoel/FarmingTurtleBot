import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class SimpleNavigator(Node):
    def __init__(self, node_name='simple_navigator'):
        super().__init__(node_name)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cli = self.create_client(SetParameters, '/controller_server/set_parameters')

    def send_goal(self, x, y, w:float|None = None):
        dontSendParam = False
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info(f"Waiting on service")
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"UGH")
            dontSendParam = True
        req = SetParameters.Request()
        

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        if w is not None:
            goal_msg.pose.pose.orientation.w = w
            param_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.25)
        else:
            goal_msg.pose.pose.orientation.w = 1.0
            param_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=6.28)

        param = Parameter(name='goal_checker.yaw_goal_tolerance', value=param_value)
        req.parameters = [param]
        if not dontSendParam:
            self.cli.call_async(req)


        self.get_logger().debug(f'Sending goal: ({x}, {y})')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().debug('Goal rejected (ugh)')
            return

        self.get_logger().debug('Goal accepted (yay)')

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