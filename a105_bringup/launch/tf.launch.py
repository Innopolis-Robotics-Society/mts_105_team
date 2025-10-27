from launch import LaunchDescription
from launch_ros.actions import Node

def static_tf(name, x,y,z, r,p,yaw, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(r), '--pitch', str(p), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child
        ]
    )

def generate_launch_description():
    tf_base_to_lidar = static_tf('tf_base_to_lidar', 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 'base_link', 'lidar_link')
    # tf_base_to_depth_opt = static_tf('tf_base_to_depth_opt', 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 'base_link', 'camera_depth_optical_frame')
    tf_base_to_imu = static_tf('tf_base_to_imu', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'base_link', 'imu_link')
    tf_base_to_depth = static_tf('tf_base_to_depth', -0.1, 0.0, 0.3, 0, 0.0, 0, 'base_link', 'camera_link')

    tf_map_to_odom = static_tf('tf_map_to_odom', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'map', 'odom')
    tf_odom_to_base = static_tf('tf_odom_to_base', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'odom', 'base_link')

    return LaunchDescription([
        tf_base_to_lidar,
        #tf_base_to_depth_opt,
        tf_base_to_imu,
        tf_base_to_depth,
        tf_odom_to_base,
        tf_map_to_odom,
    ])
