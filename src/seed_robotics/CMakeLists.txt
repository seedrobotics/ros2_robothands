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
# Install without the .py extension so ros2 launch can find it as 'hand_handle_node'
install(PROGRAMS scripts/hand_handle_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_handle_node
)

# ── Install shared resources ─────────────────────────────────────────────────
install(DIRECTORY
  launch
  config
  user_samples
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
