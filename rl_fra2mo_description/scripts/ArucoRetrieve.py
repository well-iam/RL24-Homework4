#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


def load_waypoints(file_path):
    """Load waypoints from a YAML file into a dictionary."""
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    waypoints = {}
    for waypoint in data['waypoints']:
        name = waypoint['name']
        waypoints[name] = waypoint
    return waypoints


def create_pose(transform):
    """Create a PoseStamped message from a dictionary."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = transform["position"]["x"]
    pose.pose.position.y = transform["position"]["y"]
    pose.pose.position.z = transform["position"]["z"]
    pose.pose.orientation.x = transform["orientation"]["x"]
    pose.pose.orientation.y = transform["orientation"]["y"]
    pose.pose.orientation.z = transform["orientation"]["z"]
    pose.pose.orientation.w = transform["orientation"]["w"]
    return pose


class PoseSubscriber(Node):
    """Node to subscribe to PoseStamped messages."""
    def __init__(self):
        super().__init__('ArUcoPose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',  # Topic name
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        """Callback to process incoming PoseStamped messages."""
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.get_logger().info(
            f"Received Pose: Position: ({position.x}, {position.y}, {position.z}), "
            f"Orientation: ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})"
        )


def main():
    rclpy.init()

    # Create a BasicNavigator instance
    global navigator
    navigator = BasicNavigator()

    # Load waypoints from the YAML file
    yaml_file_name = 'ArucoGoals.yaml'
    yaml_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", yaml_file_name)
    waypoints = load_waypoints(yaml_file)

    # Define the desired order of waypoint execution
    waypoint_order = ['Goal_4','Goal_5'] #'Goal_1','Goal_2','Goal_3',

    # Create the goal poses in the specified order
    goal_poses = [create_pose(waypoints[name]) for name in waypoint_order]
    print(goal_poses)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # Start following the waypoints
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    # Start the ArUco Pose subscriber
    aruco_pose_subscriber = PoseSubscriber()
    
    # This will keep the node spinning and processing messages as they arrive
    rclpy.spin(aruco_pose_subscriber)

    # Monitor navigation task completion
    i = 0
    while not navigator.isTaskComplete():
        

       
        i += 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Handle navigation timeout for demonstration
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()



    # Handle the result of the navigation
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


    aruco_pose_subscriber.destroy_node()

    exit(0)


if __name__ == '__main__':
    main()
