from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("a105_imu"), "launch", "imu.launch.py"])
        )
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            '/imu/data_raw': False,
            '/imu/data_raw': '/imu/data_raw',
            '/imu/data': '/imu/data'
        }]
    )

    return LaunchDescription([
        imu,
        imu_filter,
    ])