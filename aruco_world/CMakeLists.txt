cmake_minimum_required(VERSION 3.8)
project(aruco_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models rviz urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/camera_snapshot.py DESTINATION lib/${PROJECT_NAME})

ament_package()
