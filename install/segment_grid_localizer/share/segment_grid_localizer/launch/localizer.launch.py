from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='segment_grid_localizer',
            executable='segment_grid_localizer_node',
            name='segment_grid_localizer',
            output='screen',
            parameters=[{
                "map_topic": "/map",
                "scan_topic": "/scan",
                "odom_topic": "/odometry/filtered",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",

                # поиск вокруг одометрии
                "search_xy_max": 0.40,
                "search_yaw_max": 0.35,  # ~20°
                "step_xy_coarse": 0.05,
                "step_yaw_coarse": 0.05,
                "step_xy_fine": 0.01,
                "step_yaw_fine": 0.01,

                # фильтрация скана
                "range_min": 0.05,
                "range_max": 8.0,
                "downsample": 2,

                # поле расстояний по карте
                "occ_threshold": 95,
                "sigma_hit": 0.05,
                "penalty_unknown": 0.2,   # штраф за неизвестные клетки
                "penalty_outside": 0.0,   # игнор вне карты

                # сглаживание трансформации map->odom
                "alpha_smooth": 1.0
            }]
        )
    ])
