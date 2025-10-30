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

    lidar = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'lidar_link'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},  # unit is degress
        {'angle_crop_max': 225.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 12.0}   # unit is meter
      ]
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
        PathJoinSubstitution([FindPackageShare("a105_bringup"), "launch", "imu.launch.py"]),
    )

    ld = LaunchDescription(
        [
            declare_use_sim_time_cmd,

            wheels,
            lidar,
            #depth_camera,
            #imu,
        ]
    )


    return ld
