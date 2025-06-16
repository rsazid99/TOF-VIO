from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    args = [
        DeclareLaunchArgument('base_name', default_value='pico_flexx'),
        DeclareLaunchArgument('sensor', default_value=''),
        DeclareLaunchArgument('use_case', default_value='1'),
        DeclareLaunchArgument('automatic_exposure', default_value='true'),
        DeclareLaunchArgument('exposure_time', default_value='1000'),
        DeclareLaunchArgument('exposure_mode', default_value='1'),
        DeclareLaunchArgument('exposure_time_stream2', default_value='1000'),
        DeclareLaunchArgument('exposure_mode_stream2', default_value='1'),
        DeclareLaunchArgument('max_noise', default_value='0.07'),
        DeclareLaunchArgument('filter_level', default_value='200'),
        DeclareLaunchArgument('range_factor', default_value='2.0'),
        DeclareLaunchArgument('queue_size', default_value='2'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('base_name_tf', default_value=LaunchConfiguration('base_name')),
        DeclareLaunchArgument('machine', default_value='localhost'),
        DeclareLaunchArgument('define_machine', default_value='true'),
        DeclareLaunchArgument('nodelet_manager', default_value=LaunchConfiguration('base_name')),
        DeclareLaunchArgument('start_manager', default_value='true'),
    ]

    static_tf = Node(
        package='tf',
        executable='static_transform_publisher',
        name=[LaunchConfiguration('base_name'), '_static_tf'],
        arguments=[
            '0', '0', '0', '0', '0', '-1.57079632679489661923',
            PythonExpression([LaunchConfiguration('base_name_tf'), " + '_link'"]),
            PythonExpression([LaunchConfiguration('base_name_tf'), " + '_optical_frame'"]),
            '100'
        ],
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )

    manager = Node(
        package='nodelet',
        executable='nodelet',
        name=LaunchConfiguration('nodelet_manager'),
        arguments=['manager'],
        output='log',
        condition=IfCondition(LaunchConfiguration('start_manager'))
    )

    driver = Node(
        package='nodelet',
        executable='nodelet',
        name=[LaunchConfiguration('base_name'), '_driver'],
        arguments=[
            'load', 'pico_flexx_driver/pico_flexx_nodelet',
            LaunchConfiguration('nodelet_manager'),
            'pico_flexx/points:=/points',
            'pico_flexx/image_depth:=/image_depth',
            'pico_flexx/image_mono8:=/image_nir'
        ],
        respawn=True,
        output='log',
        parameters=[{
            'base_name': LaunchConfiguration('base_name'),
            'sensor': LaunchConfiguration('sensor'),
            'use_case': LaunchConfiguration('use_case'),
            'automatic_exposure': LaunchConfiguration('automatic_exposure'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'exposure_mode': LaunchConfiguration('exposure_mode'),
            'exposure_time_stream2': LaunchConfiguration('exposure_time_stream2'),
            'exposure_mode_stream2': LaunchConfiguration('exposure_mode_stream2'),
            'max_noise': LaunchConfiguration('max_noise'),
            'filter_level': LaunchConfiguration('filter_level'),
            'range_factor': LaunchConfiguration('range_factor'),
            'queue_size': LaunchConfiguration('queue_size'),
            'base_name_tf': LaunchConfiguration('base_name_tf'),
        }]
    )

    return LaunchDescription(args + [static_tf, manager, driver])
