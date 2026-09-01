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

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  foreach(test_name sensor_wrench_adapter launch_files)
    ament_add_test(${test_name}
      COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_${test_name}.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      TIMEOUT 900
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )
  endforeach()
endif()

ament_package()
