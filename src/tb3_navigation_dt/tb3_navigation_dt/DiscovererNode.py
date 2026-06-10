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
from custom_interfaces.action import DiscoverField
import matplotlib.pyplot as plt
import heapq

NAVIGATIONFAILURELIMIT = 100

class DiscovererNode(SimpleNavigator):
    
    def __init__(self, node_name='Discoverer'):
        super().__init__(node_name)

        self._actionServer = ActionServer(
            self,
            DiscoverField,
            'fieldDiscovery',
            self._handle_discoverField,
        )

        self._subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

        self.map = None
        self.frontier = (0.0, 0.0)
        self.failedToNavigateCounter = 0

    def _handle_discoverField(self, goal_handle):
        self._start()
        self.goal_handle = goal_handle

        rate = self.create_rate(0.2)
        while rclpy.ok() and self.goal_handle.is_active:

            if (self.map is None):
                self.get_logger().info("Waiting on map")

            if self.failedToNavigateCounter > NAVIGATIONFAILURELIMIT: 
                self.get_logger().info("Failed two many times, assuming no new frontiers")
                break

            frontier = self._calculate_frontiers()
            if frontier is None:
                return
            self.frontier = (
                frontier[0] * self.map.info.resolution + self.map.info.origin.position.x,
                frontier[1] * self.map.info.resolution + self.map.info.origin.position.y,
            )
            self.get_logger().info(f"{frontier}, {self.frontier}")

            self.send_goal(*self.frontier)

            rate.sleep()

        result = DiscoverField.Result()

        if self.failedToNavigateCounter > NAVIGATIONFAILURELIMIT:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result
    
    def _start(self):
        self.send_goal(*self.frontier)
        return
    
    def _map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def _calculate_frontiers(self):
        map = np.reshape(self.map.data, (self.map.info.height, self.map.info.width))

        startX = -round(self.map.info.origin.position.x / self.map.info.resolution)
        startY = -round(self.map.info.origin.position.y / self.map.info.resolution)
        
        queue = [(0, startY, startX)] # Cost, Y, X

        min_costs = np.full((self.map.info.height, self.map.info.width), np.inf)
        min_costs[startY, startX] = 0
        directions = [(-1, 0), (1,0), (0, -1), (0,1)]

        while queue:
            current_cost, y, x = heapq.heappop(queue)

            if map[y, x] == -1:
                return (y, x)
            
            if current_cost > min_costs[y, x]:
                continue

            for dy, dx in directions:
                ny, nx = y + dy, x + dx

                if 0 <= ny < self.map.info.height and 0 <= nx < self.map.info.width:
                    neighbor_weight = map[ny, nx] ** 2

                    step_cost = 0 if neighbor_weight == -1 else neighbor_weight
                    new_cost = current_cost + step_cost
                    
                    if new_cost < min_costs[ny, nx]:
                        min_costs[ny, nx] = new_cost
                        heapq.heappush(queue, (new_cost, ny, nx))

        return None

    @override
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        self.get_logger().info(f"{self.failedToNavigateCounter}")

        if status == 4: # TaskResult.SUCCEEDED
            self.get_logger().info(f'Docking station reached successfully!')
        else:
            self.get_logger().info(f'Failed to reach Docking station with status code: {status}')
            self.failedToNavigateCounter += 1

        if self.failedToNavigateCounter < NAVIGATIONFAILURELIMIT:
            self.send_goal(*self.frontier)

def main():
    rclpy.init()
    node = DiscovererNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()