import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult

from tb3_navigation_dt.frontier_explorer import FrontierExplorer
from tb3_navigation_dt.zigzag_planner import generate_zigzag_waypoints

STRIP_WIDTH = 0.5   # metres between zigzag strips
MARGIN = 0.3        # metres to keep away from walls during sweep
NO_FRONTIER_CONFIRMATIONS = 3   # consecutive empty checks before declaring done


def _make_pose(nav: FrontierExplorer, x: float, y: float, yaw_w: float = 1.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = yaw_w
    return pose


def _phase1_explore(explorer: FrontierExplorer):
    explorer.get_logger().info('=== Phase 1: Frontier Exploration ===')
    empty_streak = 0

    while rclpy.ok():
        rclpy.spin_once(explorer, timeout_sec=0.5)

        robot_xy = explorer.get_robot_xy()
        if robot_xy is None:
            continue

        centroids = explorer.find_frontier_centroids()

        if not centroids:
            empty_streak += 1
            explorer.get_logger().info(
                f'No frontiers ({empty_streak}/{NO_FRONTIER_CONFIRMATIONS})'
            )
            if empty_streak >= NO_FRONTIER_CONFIRMATIONS:
                explorer.get_logger().info('Arena fully mapped!')
                return
            continue

        empty_streak = 0
        rx, ry = robot_xy
        centroids.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
        tx, ty, size = centroids[0]

        explorer.get_logger().info(
            f'Frontier at ({tx:.2f}, {ty:.2f})  size={size} cells'
        )
        explorer.goToPose(_make_pose(explorer, tx, ty))

        while not explorer.isTaskComplete():
            rclpy.spin_once(explorer, timeout_sec=0.1)

        result = explorer.getResult()
        if result == TaskResult.FAILED:
            explorer.get_logger().warn('Navigation to frontier failed — trying next')


def _phase2_sweep(explorer: FrontierExplorer):
    explorer.get_logger().info('=== Phase 2: Zigzag Humidity Sweep ===')

    # Let the map settle a moment before planning
    for _ in range(20):
        rclpy.spin_once(explorer, timeout_sec=0.1)

    if explorer.map_data is None:
        explorer.get_logger().error('No map available — cannot plan sweep')
        return

    points = generate_zigzag_waypoints(explorer.map_data, STRIP_WIDTH, MARGIN)
    if not points:
        explorer.get_logger().error('Zigzag planner returned no waypoints')
        return

    explorer.get_logger().info(f'Sweeping {len(points)} waypoints')
    waypoints = [_make_pose(explorer, x, y) for x, y in points]
    explorer.followWaypoints(waypoints)

    while not explorer.isTaskComplete():
        rclpy.spin_once(explorer, timeout_sec=0.1)

    result = explorer.getResult()
    if result == TaskResult.SUCCEEDED:
        explorer.get_logger().info('Humidity sweep complete!')
    else:
        explorer.get_logger().warn(f'Sweep finished with result: {result}')


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()

    explorer.get_logger().info('Waiting for Nav2...')
    explorer.waitUntilNav2Active(localizer='slam_toolbox')
    explorer.get_logger().info('Nav2 active — starting mission')

    # Wait for first map message
    while rclpy.ok() and explorer.map_data is None:
        explorer.get_logger().info('Waiting for /map...')
        rclpy.spin_once(explorer, timeout_sec=1.0)

    _phase1_explore(explorer)
    _phase2_sweep(explorer)

    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
