#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml



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

def main():
    rclpy.init()
    
    global navigator 
    navigator = BasicNavigator()
    
    # Load waypoints from the YAML file
    
    yaml_file_name = 'mapping.yaml'
    yaml_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), "config",yaml_file_name)
 
    # Load waypoints from the YAML file
    waypoints = load_waypoints(yaml_file)

    # Define the desired order of waypoint execution
    #waypoint_order = ['Goal_1', 'Goal_2','Goal_3','Goal_4','Goal_5','Goal_6','Goal_7','Goal_8','Goal_9','Goal_10','Goal_11','Goal_12','Goal_13','Goal_14','Goal_15']

    # Define the desired order of waypoint execution
    waypoint_order = ['Goal_10','Goal_9','Goal_12','Goal_13','Goal_3','Goal_2']


    # Create the goal poses in the specified order
    goal_poses = [create_pose(waypoints[name]) for name in waypoint_order]
    
    print(goal_poses)


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()