from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('a105_imu_yes')
    params = os.path.join(pkg, 'config', 'imu_params.yaml')

    return LaunchDescription([
        Node(
            package='a105_imu_yes',
            executable='imu_node',
            name='a105_imu_yes',
            parameters=[params],
            output='screen'
        )
    ])
