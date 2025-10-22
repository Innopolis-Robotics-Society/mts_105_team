from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.0', '0.0', '0.15', '0', '0', '0', 'base_link', 'lidar_link']
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0.0', '0.0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_odom_tf',
        arguments=['0.0', '0.0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_tf_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_depth_tf',
        arguments=[ "-0.1", "0.0", "0.3",
                     "-1.5707", "0.0", "-2.15349449019"
                     , 'base_link', 'depth_camera']
    )

    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_link',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        static_tf_lidar,
        static_tf_imu,
        static_tf_odom,
        static_tf_depth,
        static_tf_base_link,
    ])