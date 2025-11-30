cmake_minimum_required(VERSION 3.8)
project(s1_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(s1_interfaces REQUIRED)

add_executable(driver_node src/driver_node.cpp)
ament_target_dependencies(driver_node rclcpp s1_interfaces)

install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
