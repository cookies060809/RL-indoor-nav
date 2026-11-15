cmake_minimum_required(VERSION 3.8)
project(safety_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(safety_node src/safety_node.cpp)
target_include_directories(safety_node PRIVATE include)
ament_target_dependencies(safety_node
  rclcpp sensor_msgs geometry_msgs std_msgs std_srvs tf2_ros)

install(TARGETS safety_node DESTINATION lib/${PROJECT_NAME})
add_executable(safety_core_check test/safety_core_check.cpp)
target_include_directories(safety_core_check PRIVATE include)
install(TARGETS safety_core_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
