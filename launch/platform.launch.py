from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_vio')

    flexx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vio_platform', 'flexx.launch.py')
        )
    )

    px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vio_platform', 'px4.launch.py')
        )
    )

    icp_params = {
        'icp/use_other_icp': 0,
        'icp/use_orig_pts': False,
        'icp/use_ransomdownsample_pts': False,
        'icp/use_salient_pts': True,
        'icp/use_robust_w': True,
        'icp/vo_mode': False,
        'icp/init_by_MC': False,
        'icp/init_by_IMU': True,
        'icp/kf_criteria': 2,
        'cam_cal_file': os.path.join(pkg_share, 'config', 'flexx.yml'),
        'icp/sailentpts_sample_count': 15000,
        'icp/target_sailentpts_num': 500,
        'icp/use_depth_grad': 1,
        'icp/use_intensity_grad': 1,
        'icp/use_edge_detector': 0,
        'icp/use_canny': 1,
        'icp/sh_backfloor': 0.01,
        'icp/sh_depth': 0.04,
        'icp/sh_grey': 80.0,
        'icp/sh_edge': 0.015,
    }

    icp_node = Node(
        package='nodelet',
        executable='nodelet',
        name='ICP_Node',
        output='screen',
        arguments=['standalone', 'nodelet_ns/NICP'],
        remappings=[
            ('/input_gt', '/gt'),
            ('/input_imu', '/imu'),
            ('/input_tof_pc', '/points'),
            ('/input_tof_nir', '/image_nir'),
            ('/input_tof_depth', '/image_depth'),
            ('/NICP/out', 'remapped_output')
        ],
        parameters=[icp_params]
    )

    static_tf = Node(
        package='tf',
        executable='static_transform_publisher',
        name='worldandmap',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map', '100']
    )

    return LaunchDescription([
        flexx_launch,
        px4_launch,
        icp_node,
        static_tf
    ])
