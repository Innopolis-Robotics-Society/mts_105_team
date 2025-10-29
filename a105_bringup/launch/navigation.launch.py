from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node  # Add this import


def generate_launch_description():

    goal_x = DeclareLaunchArgument(
        'goal_x', 
        default_value='3.5'
        )
    
    goal_y  = DeclareLaunchArgument(
        'goal_y', 
        default_value='3.5'
        )
    
    goal_yaw  = DeclareLaunchArgument(
        'goal_yaw', 
        default_value='1.57'
        )
    
    goal_frame = DeclareLaunchArgument(
        'goal_frame', 
        default_value='map'
        )
    
    map_file = DeclareLaunchArgument(
        "map_file",
        default_value="",
        )
    
    use_slam = DeclareLaunchArgument(
        "use_slam", default_value="True", 
        description="Whether run a SLAM",)
    
    declare_autostart = DeclareLaunchArgument(
        "autostart", 
        default_value="True", 
        description="Automatically startup the nav2 stack", )
    
    param_file = DeclareLaunchArgument(
         "params_file", 
         default_value=PathJoinSubstitution(
            [
                FindPackageShare("a105_navigation"),
                "config",
                "nav2_entire.yaml",
            ]
        ),
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true"
    )

    map = DeclareLaunchArgument(
        "map",
        default_value="",
    )

    # slam_cfg = PathJoinSubstitution([FindPackageShare("a105_custom_slam"), "config", "segment_grid_mapper.yaml"])
    # amcl_cfg = PathJoinSubstitution([FindPackageShare("a105_navigation"), "config", "amcl.yaml"])

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': True,
            'gain': 0.1,
           # 'fixed_frame': 'base_link'
        }],
        remappings=[
            ('imu/data_raw',  '/imu/data_raw'),
            ('imu/data',      '/imu/data'),
            #('/tf',          '/tf_imu_filter_drop')  # TF от этого узла уйдёт в «пустоту»
        ]
    )

    ekf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("a105_navigation"), "launch", "ekf.launch.py"]),
    )

    nav2_bringup = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("params_file", LaunchConfiguration("params_file")),
            ("map", LaunchConfiguration("map")),
            ("slam", LaunchConfiguration("use_slam")),
            ("autostart", LaunchConfiguration("autostart")),
        ],
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

    ld = LaunchDescription(
        [
            goal_x,
            goal_y,
            goal_yaw,
            goal_frame,
            use_sim_time,
            map,
            use_slam,
            declare_autostart,
            param_file,

            lidar_odom,
            imu_filter,
            ekf,
            TimerAction(period=0.5, actions=[nav2_bringup]),
            # goal,
        ]
    )


    return ld


    # slam = Node(
    #     package="a105_custom_slam",
    #     executable="segment_grid_mapper",
    #     name="segment_grid_mapper",
    #     output="screen",
    #     parameters=[slam_cfg],
    # )

    # amcl = Node(
    #     package="nav2_amcl",
    #     executable="amcl",
    #     name="amcl",
    #     output="screen",
    #     parameters=[amcl_cfg],
    # )

    # nav2 = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare("a105_path_planner"), "launch", "path_planner_nav2.launch.py"]),
    #     launch_arguments=[
    #         ("use_sim_time", LaunchConfiguration("use_sim_time")),
    #         ("map_file", LaunchConfiguration("map_file")),
    #         ("slam", "True")
    #     ],
    # )