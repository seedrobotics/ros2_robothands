cmake_minimum_required(VERSION 3.16)
project(seed_rh8d_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/palm_ir_adapter.py
    scripts/joint_gui_to_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
