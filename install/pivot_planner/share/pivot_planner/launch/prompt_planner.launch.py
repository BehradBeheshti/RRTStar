from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'openai_api_key',
            default_value='',
            description='Your OpenAI API key'
        ),
        
        DeclareLaunchArgument(
            'prompt',
            default_value='balanced navigation',
            description='Natural language prompt for planning behavior'
        ),
        
        DeclareLaunchArgument(
            'model',
            default_value='gpt-4',
            description='OpenAI model to use (gpt-4, gpt-3.5-turbo, etc.)'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        Node(
            package='pivot_planner',
            executable='simple_map_publisher',
            output='screen'
        ),
        
        Node(
            package='pivot_planner',
            executable='prompt_pivot_planner',
            output='screen',
            parameters=[{
                'openai_api_key': LaunchConfiguration('openai_api_key'),
                'prompt': LaunchConfiguration('prompt'),
                'model': LaunchConfiguration('model')
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
