from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_vio')

    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vio.launch.py')
        )
    )

    gt_pub = Node(
        package='tof_vio',
        executable='gt_publisher',
        name='gt',
        output='log',
        remappings=[('/gt/vicon', '/gt')]
    )

    odom_file = Node(
        package='tof_vio',
        executable='topic2file',
        name='odomfile',
        output='log',
        remappings=[('~odom', '/icp_odom')],
        parameters=[{'filepath': os.path.join(pkg_share, 'result', 'icp+text.txt')}]
    )

    gt_file = Node(
        package='tof_vio',
        executable='topic2file',
        name='gtfile',
        output='log',
        remappings=[('~odom', '/gt/odom_gt')],
        parameters=[{'filepath': os.path.join(pkg_share, 'result', 'gt.txt')}]
    )

    play_bag = ExecuteProcess(
        cmd=['rosbag', 'play', os.path.join(pkg_share, 'bag', 'handheld.bag')],
        output='screen'
    )

    return LaunchDescription([
        vio_launch,
        gt_pub,
        odom_file,
        gt_file,
        play_bag
    ])
