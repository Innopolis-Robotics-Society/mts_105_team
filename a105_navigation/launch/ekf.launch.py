from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    ekf_param_file = DeclareLaunchArgument(
        "ekf_param_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("a105_navigation"),
                "config",
                "ekf_params.yaml",
            ]
        ),
    )

    ekf = Node(package='robot_localization', executable='ekf_node',
                           name='ekf_odom', output='screen',
                           parameters=[LaunchConfiguration("ekf_param_file")])


    return LaunchDescription([
        ekf_param_file,
        ekf,
    ])