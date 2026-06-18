import numpy as np
from typing import override
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tb3_navigation_dt.simpleNavigator import SimpleNavigator
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from custom_interfaces.action import DockRobot, MapField, DiscoverField
from tb3_state_dt.enums import SystemMode
from action_msgs.msg import GoalStatus

'''
          DISCOVER
          ↓
          MAP
          ↓
FILLING ↔ IDLE ← IRRIGATING
          ↓↑       ↑
          Simulating
'''

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('Orchestrator')

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self._publisher = self.create_publisher(Int32, '/mode', qos_profile)
        self._dock_action_client = ActionClient(self, DockRobot, '/dock_robot')
        self._discover_action_client = ActionClient(self, DiscoverField, '/fieldDiscovery')
        self._map_action_client = ActionClient(self, MapField, '/map_field')

        self.get_logger().info('Orchestrator node initialized.')

        self.setMode(SystemMode.DISCOVER)

    def setMode(self, mode: SystemMode):
        self.mode = mode
        msg = Int32()
        msg.data = mode.value
        self._publisher.publish(msg)

        self.startupMode()

    def startupMode(self):
        match self.mode:
            case SystemMode.DISCOVER:
                self._discover_action_client.wait_for_server()
                goal_msg = DiscoverField.Goal()
                self._discover_action_client.send_goal_async(
                    goal_msg,
                    # feedback_callback=self.feedback_callback
                ).add_done_callback(self._onStartDiscovering)
                
                return

            case SystemMode.MAP:
                self._map_action_client.wait_for_server()
                goal_msg = MapField.Goal()
                self._map_action_client.send_goal_async(
                    goal_msg,
                    # feedback_callback=self.feedback_callback
                ).add_done_callback(self._onStartMapping)
                
                return
            
            case SystemMode.IDLE:
                # TODO: Robot needs to figure out what to do next, we need some function to decide this
                return

            case SystemMode.SIMULATING:
                # TODO: Start up action to let digital twin do what it is supposed to do @marnix900
                return

            case SystemMode.IRRIGATING:
                # TODO: Start up route following function of @frederic-cvs
                return
            
            case SystemMode.FILLING:
                # TODO: Start up a simple action/function filling up the watertank
                return
            
            case _:
                self.get_logger().warn("Not implemented")
                return

    def _onStartDiscovering(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the action server')
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.onDoneDiscovering)

    def _onStartMapping(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the action server')
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.onDoneMapping)


    def onDoneDiscovering(self, future):
        status = future.result().status

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Done discovering successfully!')
                self.setMode(SystemMode.MAP)

            case GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Discovery was canceled.')
                self.setMode(SystemMode.ERROR)

            case GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Goal was aborted')
                self.setMode(SystemMode.ERROR)

            case _:
                self.get_logger().info(f'Goal failed with status code: {status}')
                self.setMode(SystemMode.ERROR)
        

    def onDoneMapping(self, future):
        result = future.result().result
        status = future.result().status

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Done mapping successfully!')
                self.setMode(SystemMode.IDLE)

            case GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Mapping was canceled.')
                self.setMode(SystemMode.ERROR)

            case GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Goal was aborted')
                self.setMode(SystemMode.ERROR)

            case _:
                self.get_logger().info(f'Goal failed with status code: {status}')
                self.setMode(SystemMode.ERROR)

    def onDoneSimulating(self, future):
        result = future.result().result
        status = future.result().status

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Done simulating successfully!')
                # TODO: Some code to check how the simulation went
                self.setMode(SystemMode.IDLE)

            case GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Simulation was canceled.')
                self.setMode(SystemMode.ERROR)

            case GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Goal was aborted')
                self.setMode(SystemMode.ERROR)

            case _:
                self.get_logger().info(f'Goal failed with status code: {status}')
                self.setMode(SystemMode.ERROR)

    def onDoneIrrigation(self, future):
        result = future.result().result
        status = future.result().status

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Done irrigating successfully!')
                # TODO: Some code to check how the simulation went
                self.setMode(SystemMode.IDLE)

            case GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Irrigation was canceled.')
                self.setMode(SystemMode.ERROR)

            case GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Goal was aborted')
                self.setMode(SystemMode.ERROR)

            case _:
                self.get_logger().info(f'Goal failed with status code: {status}')
                self.setMode(SystemMode.ERROR)

def main():
    rclpy.init()
    node = OrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()