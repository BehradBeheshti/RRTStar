from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='pivot_planner', executable='simple_map_publisher', output='screen'),
        Node(package='pivot_planner', executable='multi_path_planner', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen'),
    ])
