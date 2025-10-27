from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('a105_mpu6500_spi_driver'),
        'config',
        'mpu6500_spi.yaml'
    )

    return LaunchDescription([
        Node(
            package='a105_mpu6500_spi_driver',
            executable='mpu6500_spi_node', 
            name='mpu6500_spi_node',
            parameters=[cfg],
            output='screen'
        )
    ])
