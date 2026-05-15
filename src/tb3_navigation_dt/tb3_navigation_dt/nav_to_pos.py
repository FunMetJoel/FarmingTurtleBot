import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Set initial pose (Optional if already set in RViz)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)

    # 2. Wait for Nav2 to be fully active
    nav.waitUntilNav2Active()

    # 3. Define a goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0  # Move 2 meters forward
    goal_pose.pose.position.y = 1.0  # Move 1 meter left
    goal_pose.pose.orientation.w = 1.0

    # 4. Go to the pose!
    nav.goToPose(goal_pose)

    # 5. Monitor the task
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f'Estimated time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s')

    # 6. Check the result
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()