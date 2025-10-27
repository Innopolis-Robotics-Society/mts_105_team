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

    wheels = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_cobra_driver"), "launch", "cobra.launch.py"]),
    )

    lidar = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_lidar"), "launch", "ld19.launch.py"]),
    )

    depth_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("orbbec_camera"), "launch", "dabai_dcw.launch.py"])
    ),
    # launch_arguments={
    #     "camera_name": "camera",
    #     "enable_color": "true",
    #     "enable_depth": "true",
    #     "depth_registration": "true",
    #     "color_width": "1280", "color_height": "720", "color_fps": "30",
    #     "depth_width": "640",  "depth_height": "480", "depth_fps": "30",
    # }.items(),
    )

    imu = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("bno055"), "launch", "bno055.launch.py"]),
    )

    ld = LaunchDescription(
        [
            declare_use_sim_time_cmd,

            wheels,
            lidar,
            depth_camera,
            #imu,
        ]
    )


    return ld
