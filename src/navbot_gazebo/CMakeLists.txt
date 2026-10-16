cmake_minimum_required(VERSION 3.8)
project(navbot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config params worlds maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS tools/verify_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
