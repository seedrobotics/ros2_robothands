cmake_minimum_required(VERSION 3.8)
project(seed_robotics)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ── Generate custom message types ───────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoneJoint.msg"
  "msg/AllJoints.msg"
  "msg/JointSetSpeedPos.msg"
  "msg/JointListSetSpeedPos.msg"
  "msg/ClearHWError.msg"
  "msg/SetStiffness.msg"
  "msg/JointListSetStiffness.msg"
  "msg/SetShutdownCond.msg"
  "msg/LoneMainBoard.msg"
  "msg/AllMainBoards.msg"
  DEPENDENCIES std_msgs
)

# ── Install Python executable scripts ───────────────────────────────────────
# Install without the .py extension so ros2 run/launch can find them
install(PROGRAMS scripts/hand_handle_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_handle_node
)

#install(PROGRAMS scripts/glove_ros2_node.py
#  DESTINATION lib/${PROJECT_NAME}
#  RENAME glove_ros2_node
#)
#
#install(PROGRAMS scripts/aruco_multicam_pose_ros2.py
#  DESTINATION lib/${PROJECT_NAME}
#  RENAME aruco_multicam_pose_node
#)

# ── Install shared resources ─────────────────────────────────────────────────
install(DIRECTORY
  launch
  config
  user_samples
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
