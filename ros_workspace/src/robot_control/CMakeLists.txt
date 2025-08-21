cmake_minimum_required(VERSION 3.10)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)


add_executable(manual src/manual_node.cpp)
ament_target_dependencies(manual rclcpp geometry_msgs std_msgs)

add_executable(listener src/listener_node.cpp)
ament_target_dependencies(listener rclcpp nav_msgs sensor_msgs tf2)

install(TARGETS
  manual
  DESTINATION lib/${PROJECT_NAME})

ament_package()
