from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pivot_pkg = get_package_share_directory('pivot_planner')
    clearpath_pkg = get_package_share_directory('clearpath_gz')
    
    use_prompt = LaunchConfiguration('use_prompt')
    openai_key = LaunchConfiguration('openai_api_key')
    prompt_text = LaunchConfiguration('prompt')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_prompt', default_value='true'),
        DeclareLaunchArgument('openai_api_key', default_value=''),
        
        
        # Launch Clearpath Jackal in Gazebo with your world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(clearpath_pkg, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'world': 'park_demo',
            }.items()
        ),
        
        # TF (map -> odom)
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # Map Publisher
        Node(
            package='pivot_planner', 
            executable='simple_map_publisher', 
            output='screen'
        ),
        
        # PIVOT Planners
        Node(
            package='pivot_planner', 
            executable='multi_path_planner', 
            output='screen',
            condition=UnlessCondition(use_prompt)
        ),
        
        Node(
            package='pivot_planner', 
            executable='prompt_pivot_planner', 
            output='screen',
            parameters=[{
                'openai_api_key': openai_key, 
                'prompt': prompt_text, 
                'model': 'gpt-4'
            }],
            condition=IfCondition(use_prompt)
        ),
        
        # Path Follower (with Jackal topics)
        TimerAction(
            period=5.0,
            actions=[Node(
                package='pivot_planner', 
                executable='path_follower', 
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'lookahead_distance': 0.4,
                    'linear_speed': 0.3,
                    'angular_gain': 2.0,
                    'waypoint_tolerance': 0.2
                }],
                remappings=[
                    ('/odom', '/j100_0000/platform/odom'),
                    ('/cmd_vel', '/j100_0000/platform/cmd_vel')
                ]
            )]
        ),
        
        # RViz
        Node(
            package='rviz2', 
            executable='rviz2', 
            output='screen'
        ),
    ])