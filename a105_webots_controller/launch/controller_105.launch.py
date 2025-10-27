from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a105_webots_controller',
            executable='sender',
            
        )
    ])
