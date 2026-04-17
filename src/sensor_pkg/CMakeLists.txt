cmake_minimum_required(VERSION 3.8)
project(sensor_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoneSensor.msg"
  "msg/AllSensors.msg"
  "msg/SensorUserCommand.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  script/read_publish_sensor_node.py
  user_example/user_sample_1_read_values.py
  user_example/user_sample_2_abs_pitch_yaw.py
  user_example/user_sample_3_setfreq_calibrate.py
  user_example/user_sample_4_diagnosis_request.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
