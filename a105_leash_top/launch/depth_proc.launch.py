from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node  # Add this import




def generate_launch_description():

    edge_depth_to_scan = Node(
        package='a105_leash_top',
        executable='edge_depth_to_scan',
        name='edge_depth_to_scan',
        parameters=[{
            'depth_topic': '/camera/depth/image_raw',
            'info_topic':  '/camera/depth/camera_info',
            'target_frame':'base_link',
            'range_min': 1.1,
            'range_max': 8.0,
            'scan_res': 720,
            'canny_lo': 20,
            'canny_hi': 60,
            'exclude_width_px': 140,
            'exclude_height_px': 90,
        }]
    )

    depth_proc = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        output='screen',
        remappings=[
            ('image_rect','/camera/depth/depth_edges'),
            ('camera_info','/camera/depth/camera_info'),
            ('points','/camera/depth/points'),
        ],
        parameters=[{'use_sim_time': False,
            'qos_overrides./camera/depth/points.subscription.reliability': 'best_effort',
            'qos_overrides./scan.publisher.reliability': 'reliable',
            }],
    )

    pcl2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in','/camera/depth/points'),
        ],
        parameters=[{
            'target_frame': 'base_link',   # хотим скан в базе
            'range_min': 0.01,
            'range_max': 80.0,
            # фильтр пола: оставь разумно, не ±10м
            'min_height': -4.0,
            'max_height': 4.0,
            # опционально:
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.0174533,  # 1°
            'use_inf': True,
            'transform_tolerance': 0.05,
            'qos_overrides./camera/depth/points.subscription.reliability': 'best_effort',
            'qos_overrides./scan.publisher.reliability': 'reliable',
        }],
    )

    line = Node(package='a105_leash_top', executable='rgbd_to_cmd', name='rgbd_to_cmd')
    # line = Node(
    #     package='a105_leash',
    #     executable='rgbd_to_cmd',
    #     name='a105_leash',
    #     parameters=[{
    #                 "enable_debug": True,

    #                 "hsv_S_max": 60,
    #                 "hsv_V_min": 180,
    #                 "morph_kernel": 5,
    #                 "roi_ymin_frac": 0.05,
    #                 "roi_ymax_frac": 0.76,
    #                 "near_cut_m": 0.30,

    #                 "step_px": 8,
    #                 "min_width_px": 12,

    #                 "erode_kernel": 7,   # нечётный, пикс
    #                 "erode_iter":   3,   # итерации

    #                 "lookahead_m": 0.60,
    #                 "min_v": 0.05,
    #                 "max_v": 0.23,

    #                 "w_gain": 2.5,
    #                 "w_max": 2.5,
    #                 "Ld_min": 0.25,
    #                 "Ld_alpha_k": 0.8,
    #                 "v_alpha_k": 1.4,
    #         }],
    #     )
    #line = Node(package='a105_leash_top', executable='stenka', name='stenka')


    return LaunchDescription(
        [
            #edge_depth_to_scan,
            #depth_proc, 
            #pcl2scan,
            line,
        ]
    )