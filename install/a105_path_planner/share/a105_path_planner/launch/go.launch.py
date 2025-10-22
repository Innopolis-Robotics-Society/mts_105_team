from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    goal_frame = LaunchConfiguration('goal_frame')

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='0.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),   # радианы
        DeclareLaunchArgument('goal_frame', default_value='map'),

        Node(
            package='a105_path_planner',
            executable='nav2_goal_client',
            name='nav2_goal_client',
            parameters=[{
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_yaw': goal_yaw,
                'goal_frame': goal_frame,
            }]
        ),
    ])
