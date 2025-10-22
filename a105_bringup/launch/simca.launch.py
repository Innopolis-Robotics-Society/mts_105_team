from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node  # Add this import


def generate_launch_description():

    goal_x = DeclareLaunchArgument('goal_x', default_value='3.5')
    goal_y  = DeclareLaunchArgument('goal_y', default_value='3.5')
    goal_yaw  = DeclareLaunchArgument('goal_yaw', default_value='1.57')
    goal_frame = DeclareLaunchArgument('goal_frame', default_value='map')
    rviz_cfg = PathJoinSubstitution([FindPackageShare('a105_bringup'), 'rviz', 'simca.rviz'])
    map_file = DeclareLaunchArgument("map_file", default_value="",)
    slam_cfg = PathJoinSubstitution([FindPackageShare("a105_custom_slam"), "config", "segment_grid_mapper.yaml"])
    amcl_cfg = PathJoinSubstitution([FindPackageShare("a105_bringup"), "config", "amcl.yaml"])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation (Gazebo) clock if true"
    )

    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_path_planner"), "launch", "path_planner_nav2.launch.py"]),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("map_file", LaunchConfiguration("map_file")),
            ("slam", "False")
        ],
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
        executable= 'webots_bridge_node',
        name = "wbtg_combo_pub",
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    rul = Node(
        package= 'rqt_robot_steering',
        executable= 'rqt_robot_steering',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    goal_node = Node(
        package='a105_path_planner',
        executable='nav2_goal_client',
        name='nav2_goal_client',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'goal_frame': LaunchConfiguration('goal_frame'),
        }]
    )

    goal = TimerAction(period=4.0, actions=[goal_node])

    rviz2 = Node(
        package= 'rviz2',
        executable= 'rviz2',
        arguments=['-d', rviz_cfg],
    )

    lidar_odom = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        parameters=[{
            'publish_tf': False,
            'publish_odom': '/lidar/odom',
        }]
    )

    slam = Node(
        package="a105_custom_slam",
        executable="segment_grid_mapper",
        name="segment_grid_mapper",
        output="screen",
        parameters=[slam_cfg],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_cfg],
    )
    ld = LaunchDescription(
        [
            goal_x,
            goal_y,
            goal_yaw,
            goal_frame,
            declare_use_sim_time_cmd,
            map_file,
            tf,
            bridge,
            controller,
            lidar_odom,
            ekf,
            TimerAction(period=0.5, actions=[nav2]),
            rul,
            rviz2,
            slam,
            amcl,
            goal,
        ]
    )


    return ld
