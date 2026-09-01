cmake_minimum_required(VERSION 3.16)
project(seed_rh8d_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rh8d_coupling_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    validate_urdf
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/validate_urdf.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  foreach(test_name coupling_node launch_files)
    ament_add_test(${test_name}
      COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_${test_name}.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      TIMEOUT 600
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )
  endforeach()
endif()

ament_package()
