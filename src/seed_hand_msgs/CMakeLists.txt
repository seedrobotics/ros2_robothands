cmake_minimum_required(VERSION 3.8)
project(seed_hand_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

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

ament_package()
