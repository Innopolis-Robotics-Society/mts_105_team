from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node  # Add this import


def generate_launch_description():


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation (Gazebo) clock if true"
    )

    controller = Node(
            package='a105_webots_controller',
            executable='sender',
            arguments=['--ros-args', '--log-level', 'error']
        )

    bridge = Node(
        package= 'a105_webots_bridge',
        executable= 'webots_bridge_node',
        name = "wbtg_combo_pub",
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )


    ld = LaunchDescription(
        [
            declare_use_sim_time_cmd,

            bridge,
            controller,
        ]
    )

    return ld
