cmake_minimum_required(VERSION 3.16)
project(rokoko_glove_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# The node and the glove modules land in one directory: the modules import
# each other flatly (`from hand_joints import ...`), so they have to sit
# beside whatever imports them.
install(PROGRAMS scripts/glove_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME glove_node
)

install(PROGRAMS scripts/glove_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME glove_calibrate
)

# The glove library. It lands beside the scripts because its modules import
# each other flatly, so whatever imports them has to sit in the same
# directory. rh8d_calibration.json travels with it when it exists.
file(GLOB GLOVE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/*.py")
install(
  FILES ${GLOVE_LIB}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # Needs no glove and no solver: poses are synthesised.
  ament_add_test(glove_mapping
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_glove_mapping.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
