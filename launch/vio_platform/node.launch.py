from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('fcu_url'),
        DeclareLaunchArgument('gcs_url'),
        DeclareLaunchArgument('tgt_system'),
        DeclareLaunchArgument('tgt_component'),
        DeclareLaunchArgument('pluginlists_yaml'),
        DeclareLaunchArgument('config_yaml'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
    ]

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        clear_params=True,
        output=LaunchConfiguration('log_output'),
        arguments=['mavros/imu/data:=/imu'],
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': LaunchConfiguration('tgt_system'),
                'target_component_id': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            },
            LaunchConfiguration('pluginlists_yaml'),
            LaunchConfiguration('config_yaml')
        ]
    )

    return LaunchDescription(args + [mavros])
