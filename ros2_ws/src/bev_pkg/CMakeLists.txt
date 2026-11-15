cmake_minimum_required(VERSION 3.8)
project(bev_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(project_msgs REQUIRED)

add_library(bev_core src/bev_core.cpp)
target_include_directories(bev_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(bev_node src/bev_node.cpp)
target_link_libraries(bev_node bev_core)
ament_target_dependencies(bev_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs project_msgs)

add_executable(bev_fixture_check test/bev_fixture_check.cpp)
target_link_libraries(bev_fixture_check bev_core)

install(TARGETS bev_node bev_fixture_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
