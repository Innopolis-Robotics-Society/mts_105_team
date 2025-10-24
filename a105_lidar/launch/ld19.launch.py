from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('a105_lidar'), 'config', 'ld19.yaml')
    return LaunchDescription([
        Node(
            package='a105_lidar',
            executable='ld19_to_laserscan',
            name='ld19_to_laserscan',
            output='screen',
            parameters=[cfg],
        )
    ])
