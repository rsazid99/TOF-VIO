from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tof_vio')

    icp_params = {
        'icp/use_other_icp': 0,
        'icp/use_orig_pts': False,
        'icp/use_ransomdownsample_pts': False,
        'icp/use_salient_pts': True,
        'icp/use_robust_w': True,
        'icp/vo_mode': False,
        'icp/init_by_MC': True,
        'icp/init_by_IMU': False,
        'icp/kf_criteria': 2,
        'cam_cal_file': os.path.join(pkg_share, 'config', 'flexx.yml'),
        'icp/sailentpts_sample_count': 15000,
        'icp/target_sailentpts_num': 600,
        'icp/use_depth_grad': 1,
        'icp/use_intensity_grad': 1,
        'icp/use_edge_detector': 0,
        'icp/use_canny': 1,
        'icp/sh_backfloor': 0.01,
        'icp/sh_depth': 0.06,
        'icp/sh_grey': 70.0,
        'icp/sh_edge': 0.015,
    }

    icp_node = Node(
        package='tof_vio',
        executable='icp_node',
        name='icp_node',
        output='screen',
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

    eskf_node = Node(
        package='tof_vio',
        executable='eskf',
        name='eskf',
        output='screen',
        remappings=[('imu', '/imu'), ('vo', '/icp_odom')],
        parameters=[{
            'eskf/ng': 10.0,
            'eskf/na': 10.0,
            'eskf/nbg': 10.0,
            'eskf/nba': 10.0,
            'eskf/vo_p': 0.1,
            'eskf/vo_q': 0.1,
            'eskf/vo_delay_ms': 0.0,
        }]
    )

    return LaunchDescription([icp_node, eskf_node])
