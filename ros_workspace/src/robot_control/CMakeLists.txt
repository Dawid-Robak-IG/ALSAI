cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0167 OLD)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(manual src/manual_node.cpp)
ament_target_dependencies(manual rclcpp geometry_msgs std_msgs)

add_executable(listener src/listener_node.cpp)
ament_target_dependencies(listener rclcpp nav_msgs sensor_msgs tf2 geometry_msgs)

add_library(pose_bridge SHARED src/pose_bridge.cpp)
ament_target_dependencies(pose_bridge gazebo rclcpp geometry_msgs)
target_link_libraries(pose_bridge ${GAZEBO_LIBRARIES})

add_library(tf2_plugin SHARED src/tf2_plugin.cpp)
ament_target_dependencies(tf2_plugin gazebo rclcpp geometry_msgs nav_msgs tf2_ros tf2)
target_link_libraries(tf2_plugin ${GAZEBO_LIBRARIES})

install(TARGETS
  manual
  listener
  pose_bridge
  tf2_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_package()
