from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_vio')

    args = [
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('usb_port_id', default_value=''),
        DeclareLaunchArgument('device_type', default_value='t265'),
        DeclareLaunchArgument('json_file_path', default_value=''),
        DeclareLaunchArgument('camera', default_value='camera'),
        DeclareLaunchArgument('tf_prefix', default_value=LaunchConfiguration('camera')),
        DeclareLaunchArgument('fisheye_width', default_value='848'),
        DeclareLaunchArgument('fisheye_height', default_value='800'),
        DeclareLaunchArgument('enable_fisheye1', default_value='false'),
        DeclareLaunchArgument('enable_fisheye2', default_value='false'),
        DeclareLaunchArgument('fisheye_fps', default_value='30'),
        DeclareLaunchArgument('gyro_fps', default_value='200'),
        DeclareLaunchArgument('accel_fps', default_value='62'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('enable_accel', default_value='false'),
        DeclareLaunchArgument('enable_sync', default_value='false'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('initial_reset', default_value='true'),
        DeclareLaunchArgument('unite_imu_method', default_value=''),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
    ]

    realsense_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch/includes/nodelet.launch.xml')
        ),
        launch_arguments={
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'serial_no': LaunchConfiguration('serial_no'),
            'usb_port_id': LaunchConfiguration('usb_port_id'),
            'device_type': LaunchConfiguration('device_type'),
            'json_file_path': LaunchConfiguration('json_file_path'),
            'enable_sync': LaunchConfiguration('enable_sync'),
            'fisheye_width': LaunchConfiguration('fisheye_width'),
            'fisheye_height': LaunchConfiguration('fisheye_height'),
            'enable_fisheye1': LaunchConfiguration('enable_fisheye1'),
            'enable_fisheye2': LaunchConfiguration('enable_fisheye2'),
            'fisheye_fps': LaunchConfiguration('fisheye_fps'),
            'gyro_fps': LaunchConfiguration('gyro_fps'),
            'accel_fps': LaunchConfiguration('accel_fps'),
            'enable_gyro': LaunchConfiguration('enable_gyro'),
            'enable_accel': LaunchConfiguration('enable_accel'),
            'linear_accel_cov': LaunchConfiguration('linear_accel_cov'),
            'initial_reset': LaunchConfiguration('initial_reset'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'publish_odom_tf': LaunchConfiguration('publish_odom_tf')
        }.items()
    )

    odom2pose = Node(
        package='tof_vio',
        executable='odom2posestamp',
        name='odom2pose',
        output='log',
        parameters=[{
            'odom_in': '/camera/odom/sample',
            'pose_out': '/gt'
        }]
    )

    gt_pub = Node(
        package='tof_vio',
        executable='gt_publisher',
        name='gt',
        output='log',
        remappings=[('/gt/vicon', '/gt')]
    )

    return LaunchDescription(args + [realsense_include, odom2pose, gt_pub])
