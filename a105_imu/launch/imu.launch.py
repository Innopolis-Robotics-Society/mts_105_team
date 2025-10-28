from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('a105_imu'),
        'config',
        'imu.yaml'
    )

    return LaunchDescription([
        Node(
            package='a105_imu',
            executable='imu_node', 
            name='imu_node',
            parameters=[cfg],
            output='screen'
        )
    ])
