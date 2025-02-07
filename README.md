# Mobile Robot Navigation and Mapping (Homework4)

## :package: About
Goals:
- Control a mobile robot for autonomous navigation in Gazebo.
- Map the environment using SLAM.
- Perform vision-based navigation with ArUco markers.
- Use Nav2 Framework to achieve waypoint navigation, explore mapping techniques, and tune parameters for SLAM and exploration.

Tests and results are included in the report file, comparing navigation performance under different settings, with visualizations of robot trajectories and mapping accuracy. Code and simulation videos are available on GitHub and YouTube. 
This package contains the developed code for the fourth homework of the Robotics Lab 2024/25 Course. The authors of the package are:
William Notaro, Chiara Panagrosso, Salvatore Piccolo, Roberto Rocco.

The folder contains, in addition to all the necessary packages for proper functionality, a subfolder named 'Attempts'. This subfolder includes the 4 attempts required in point 3 of the homework, with the respective parameter values set for each attempt. For completeness, the execution time and the accuracy achieved for each mapping are also provided.

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder (only works if the destination folder is empty), use:
```
git clone https://github.com/well-iam/RL24-Homework4 .
```
Alternatively, use:
```
git clone https://github.com/well-iam/RL24-Homework4
```

Build the packages with
```
colcon build
```
Source the setup files
```
source ~/ros2_ws/install/setup.bash
```

## :white_check_mark: Usage
### 1. Autonomous navigation task
Run the simulation environment through the following launch file (by default the Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button):
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
Run the RViz visualization tool and the publisher nodes through the following launch file:
```
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```
Then bring up the Nav2 package and start the SLAM Toolbox through the unified "fra2mo_run" launch file:
```
ros2 launch rl_fra2mo_description fra2mo_run.launch.py
```
#### :bangbang: Attention
Before proceeding, make sure that the fra2mo robot has spawned in the correct pose, as we observed that the robot's position is not always correctly initialized. If not, try stopping the display_fra2mo and/or the fra2mo_run launch files and re-running them.
At the time of writing, the correct spawning of the robot seems a random feature of the code execution, although it may be related to the "Back in time jumps" performed during the simulation.

Once ensured that the environment is correctly set up, launch the follow_waypoints.py scripts to see fra2mo move around the map:
```
ros2 run rl_fra2mo_description follow_waypoints.py
```

### 2. Environment mapping tuning the navigation stack's parameters
Run the simulation environment through the following launch file (by default the Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button):
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
Run the RViz visualization tool and the publisher nodes through the following launch file:
```
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```
Then bring up the Nav2 package and start the SLAM Toolbox through the unified "fra2mo_run" launch file, specifying not to load a map (the default behaviour of the launch file) by passing as an input parameter the path to an example slam configuration file:
```
ros2 launch rl_fra2mo_description fra2mo_run.launch.py slam_params_file:=/home/user/ros2_ws/src/rl_fra2mo_description/config/slam_no_map.yaml
```
#### :bangbang: Attention: As pointed out in the 'Autonomous navigation task' section, make sure that the robot spawns in the correct pose.

Then run the map_everything.py script which makes fra2mo to explore the map based on the specified config file:    
```
ros2 run rl_fra2mo_description map_everything.py
```

### 3. Vision-based navigation of the mobile platform
Run the simulation environment through the following launch file (by default the Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button):
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
Run the RViz visualization tool and the publisher nodes through the following launch file:
```
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```
Then bring up the Nav2 package and start the SLAM Toolbox through the unified "fra2mo_run" launch file, specifying the aruco_retrieve flag to true. In this way, the ROS Vision framework is brought up and a static broadcaster which sends the transform from the 'map' frame to the 'aruco' frame is executed:
```
ros2 launch rl_fra2mo_description fra2mo_run.launch.py aruco_retrieve:=true
```
#### :bangbang: Attention: As pointed out in the 'Autonomous navigation task' section, make sure that the robot spawns in the correct pose.

Then run the ArucoRetrieve script which causes fra2mo to approach Obstacle 9, trigger the ArUco marker recognition, broadcast its pose and then making the robot return to its initial position:
```
ros2 run rl_fra2mo_description ArucoRetrieve.py
```
