cmake_minimum_required(VERSION 3.8)
project(seed_hand_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/sensor_wrench_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_wrench_adapter
)

ament_package()
