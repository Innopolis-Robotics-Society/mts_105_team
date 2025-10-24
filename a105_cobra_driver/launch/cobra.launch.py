from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('a105_cobra_driver'), 'config', 'cobra.yaml')
    return LaunchDescription([
        Node(
            package='a105_cobra_driver',
            executable='cmdvel_to_cobra',
            name='cmdvel_to_cobra',
            output='screen',
            parameters=[cfg],
        )
    ])
