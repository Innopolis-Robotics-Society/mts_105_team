from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    goal_x = DeclareLaunchArgument('goal_x', default_value='0.0')
    goal_y  = DeclareLaunchArgument('goal_y', default_value='2.75')
    goal_yaw  = DeclareLaunchArgument('goal_yaw', default_value='3.14')
    goal_frame = DeclareLaunchArgument('goal_frame', default_value='map')
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False", description="Use simulation (Gazebo) clock if true")
    use_sim = DeclareLaunchArgument("use_sim", default_value="False", description="Use simulation Webots")
    rviz_cfg = PathJoinSubstitution([FindPackageShare('a105_bringup'), 'rviz', 'rpi.rviz'])
    map_file = DeclareLaunchArgument("map_file", default_value="",)
    

    tf = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("a105_bringup"), 
            "launch", 
            "tf.launch.py"
        ]),
    )


    simca = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("a105_bringup"),
            "launch",
            "simca.launch.py"
        ]),
        condition=IfCondition(
            PythonExpression([
                LaunchConfiguration('use_sim'),
            ])
        ),
    )

    hardware = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("a105_bringup"),
            "launch",
            "hardware.launch.py"
        ]),
        condition=IfCondition(
            PythonExpression([
                LaunchConfiguration('use_sim'),
                ' != True'
            ])
        ),
    )

    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("a105_bringup"),
            "launch",
            "navigation.launch.py"
        ]),
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
        parameters=[{'use_sim_time': False}]
    )

    ld = LaunchDescription(
        [
            use_sim,
            goal_x,
            goal_y,
            goal_yaw,
            goal_frame,
            use_sim_time,
            map_file,

            tf,
            simca,
            hardware,
            navigation,

            #rul,
            rviz2,
        ]
    )


    return ld
