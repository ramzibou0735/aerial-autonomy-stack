from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autopilot', default_value='px4', description='Type of autopilot (px4 or ardupilot)'),
        DeclareLaunchArgument('drone_id', default_value='1', description='Id of the drone'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use gz_sim clock if true'),
        
        Node(
            package='state_sharing',
            executable='state_sharing_node',
            name='state_sharing',
            output='screen',
            parameters=[{
                'autopilot': LaunchConfiguration('autopilot'),
                'drone_id': LaunchConfiguration('drone_id'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
