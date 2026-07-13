cmake_minimum_required(VERSION 3.8)
project(go2_tf_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    go2_tf_utils/cloud_transformer.py
    go2_tf_utils/timestamp_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

target_include_directories(pointcloud_to_laserscan PUBLIC
  include
)

target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
ament_target_dependencies(pointcloud_to_laserscan
  "laser_geometry"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)
rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

install(TARGETS
  pointcloud_to_laserscan
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

add_executable(cloud_accumulation
  src/cloud_accumulation.cpp
)
ament_target_dependencies(cloud_accumulation
  "laser_geometry"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)

install(TARGETS
  cloud_accumulation
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(odom_base_tf src/odom_base_tf.cpp)

ament_target_dependencies(odom_base_tf
  "rclcpp"
  "nav2_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

install(TARGETS
  odom_base_tf
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(livox_stabilized_tf src/livox_stabilized_tf.cpp)
ament_target_dependencies(livox_stabilized_tf
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)
install(TARGETS
  livox_stabilized_tf
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(livox_to_ptcloud src/livox_to_ptcloud.cpp)
ament_target_dependencies(livox_to_ptcloud
  livox_ros_driver2 rclcpp sensor_msgs
)
install(TARGETS
  livox_to_ptcloud
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
