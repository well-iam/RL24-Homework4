from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    aruco_dir = FindPackageShare('aruco_ros')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    aruco_retrieve = LaunchConfiguration('aruco_retrieve') 

    #file explore.yaml
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'explore.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    #tempo di gazebo
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    #variabile per estrapolare o meno la posa dell'aruco
    declare_aruco_cmd = DeclareLaunchArgument(
        'aruco_retrieve',
        default_value='false',
        description='If true we will try to retrieve he ArUco marker pose with respect to the map frame',
    )
    
    #quale file yaml usare per lo SLAM
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'slam.yaml']), #slam_no_map.yaml
        description='Specification if map is given or not',
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_slam.launch.py'])
            
        ),
        launch_arguments={'use_sim_time': use_sim_time, 
                          'slam_params_file': slam_params_file}.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )
 
    aruco_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([aruco_dir, 'launch', 'single.launch.py'])
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
            Node(
                package='aruco_transform',
                executable='static_aruco_tf2_broadcaster',
            ),
        ],
        condition=IfCondition(aruco_retrieve),
    )



    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            declare_slam_params_file_cmd,
            declare_aruco_cmd,
            slam_launch,
            nav2_bringup_launch,
            aruco_group,
        ]
    )
