cmake_minimum_required(VERSION 3.16)
project(rh8d_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/mimic_joint_clamper.py
    scripts/palm_ir_adapter.py
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
endif()

ament_package()
