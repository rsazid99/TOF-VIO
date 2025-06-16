from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_vio')

    args = [
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyUSB0:921600'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('log_output', default_value='log'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
    ]

    node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vio_platform', 'node.launch.py')
        ),
        launch_arguments={
            'pluginlists_yaml': os.path.join(pkg_share, 'launch', 'vio_platform', 'px4_pluginlists.yaml'),
            'config_yaml': os.path.join(pkg_share, 'launch', 'vio_platform', 'px4_config.yaml'),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'log_output': LaunchConfiguration('log_output'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'respawn_mavros': LaunchConfiguration('respawn_mavros')
        }.items()
    )

    return LaunchDescription(args + [node_launch])
