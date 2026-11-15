cmake_minimum_required(VERSION 3.8)
project(project_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz models scripts DESTINATION share/${PROJECT_NAME})

ament_package()
