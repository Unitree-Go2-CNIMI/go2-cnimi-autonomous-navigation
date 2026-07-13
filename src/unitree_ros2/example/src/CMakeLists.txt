cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# find_package(unitree_go2_nav_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(go2_msgs REQUIRED)

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
# add_executable(low_level_ctrl_hg src/h1-2/lowlevel/low_level_ctrl_hg.cpp src/common/motor_crc_hg.cpp)
# add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_low_state_hg src/read_low_state_hg.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
add_executable(high_level_ctrl src/high_level_ctrl.cpp src/common/ros2_sport_client.cpp) #unitree_go2_nav_interfaces
add_executable(execute_nav2 src/execute_nav2.cpp src/common/ros2_sport_client.cpp) #unitree_go2_nav_interfaces
add_executable(stand_up src/stand_up.cpp src/common/ros2_sport_client.cpp)
add_executable(stand_down src/stand_down.cpp src/common/ros2_sport_client.cpp)
add_executable(emergency_stop src/emergency_stop.cpp src/common/ros2_sport_client.cpp)
#add_executable(record_bag src/record_bag.cpp)
add_executable(sport_mode_ctrl src/sport_mode_ctrl.cpp src/common/ros2_sport_client.cpp)
add_executable(calibrate_imu src/calibrate_imu.cpp src/common/ros2_sport_client.cpp)
add_executable(trajectory_viz src/trajectory_viz.cpp)

ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
# ament_target_dependencies(low_level_ctrl_hg ${DEPENDENCY_LIST})
# ament_target_dependencies(g1_low_level_example ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state_hg ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
#ament_target_dependencies(record_bag ${DEPENDENCY_LIST})
ament_target_dependencies(sport_mode_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(high_level_ctrl ${DEPENDENCY_LIST} geometry_msgs)
ament_target_dependencies(execute_nav2 ${DEPENDENCY_LIST} geometry_msgs go2_msgs)
ament_target_dependencies(stand_up ${DEPENDENCY_LIST} geometry_msgs)
ament_target_dependencies(stand_down ${DEPENDENCY_LIST} geometry_msgs)
ament_target_dependencies(emergency_stop ${DEPENDENCY_LIST} geometry_msgs)
ament_target_dependencies(calibrate_imu ${DEPENDENCY_LIST} sensor_msgs geometry_msgs)
ament_target_dependencies(trajectory_viz ${DEPENDENCY_LIST} geometry_msgs nav_msgs)


install(TARGETS 
        low_level_ctrl
        # low_level_ctrl_hg
        # g1_low_level_example
        read_low_state
        read_low_state_hg
        read_motion_state
        read_wireless_controller
        sport_mode_ctrl
        high_level_ctrl
        # execute_nav2
        # stand_up
        # stand_down
        emergency_stop
        trajectory_viz
        calibrate_imu
        DESTINATION)

install(TARGETS execute_nav2 stand_up stand_down
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
