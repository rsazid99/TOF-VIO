cmake_minimum_required(VERSION 3.5)
project(tof_vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  src
  src/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(icp_node
  src/vo/node_vo.cpp
  src/vo/salientpts.cpp
  src/vo/icp.cpp
  src/vo/tof_frame.cpp
)
ament_target_dependencies(icp_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs image_transport cv_bridge PCL OpenCV Eigen3 message_filters pcl_conversions tf2 tf2_ros)

add_executable(ekf src/ekf/node_ekf.cpp src/ekf/ekf_model.cpp)
ament_target_dependencies(ekf rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs Eigen3 message_filters)

add_executable(eskf src/eskf/node_eskf.cpp src/eskf/eskf_imu.cpp)
ament_target_dependencies(eskf rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs Eigen3 message_filters)

add_executable(gt_publisher src/tools/node_gt_publisher.cpp)
ament_target_dependencies(gt_publisher rclcpp nav_msgs geometry_msgs)

add_executable(topic2file src/tools/node_odomtopic2file.cpp)
ament_target_dependencies(topic2file rclcpp nav_msgs)

add_executable(odom2posestamp src/tools/node_odom2posestamp.cpp)
ament_target_dependencies(odom2posestamp rclcpp nav_msgs geometry_msgs)

add_executable(vio_broadcaster src/tools/node_tf_broadcaster.cpp)
ament_target_dependencies(vio_broadcaster rclcpp tf2_ros sensor_msgs geometry_msgs)

install(TARGETS icp_node ekf eskf gt_publisher topic2file odom2posestamp vio_broadcaster
  DESTINATION lib/${PROJECT_NAME})

ament_package()
