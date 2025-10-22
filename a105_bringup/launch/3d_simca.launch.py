from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node  # Add this import


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution([FindPackageShare('a105_bringup'), 'rviz', '3d_simca2.rviz'])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation (Gazebo) clock if true"
    )

    tf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_bringup"), "launch", "tf.launch.py"]),
    )

    ekf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_navigation"), "launch", "ekf.launch.py"]),
    )

    controller = Node(
            package='a105_controller',
            executable='sender',
            arguments=['--ros-args', '--log-level', 'error']
        )

    bridge = Node(
        package= 'a105_webots_bridge',
        executable= '3d_webots_bridge_node',
        name = "d3_wbtg_combo_pub",
        output='screen',
        #arguments=['--ros-args', '--log-level', 'error']
    )

    rul = Node(
        package= 'rqt_robot_steering',
        executable= 'rqt_robot_steering',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    rviz2 = Node(
        package= 'rviz2',
        executable= 'rviz2',
        arguments=['-d', rviz_cfg],
    )

    depth_image_proc = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_leash_top"), "launch", "depth_proc.launch.py"]),
    )

    ld = LaunchDescription(
        [
            declare_use_sim_time_cmd,
            tf,
            bridge,
            controller,
            depth_image_proc,
            ekf,
            rul,
            rviz2,
        ]
    )


    return ld
