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
    scripts/driver_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)

  # Tests that need neither Gazebo nor hardware.
  foreach(test_name driver_interface interface_parity launch_files)
    ament_add_test(${test_name}
      COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_${test_name}.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      TIMEOUT 900
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )
  endforeach()

  # The simulation tests start gz-sim and take minutes each, so they are
  # opt-in; they also need ros_gz and gz_ros2_control, which the package
  # keeps out of its hard dependencies:
  #
  #   colcon build --cmake-args -DSEED_SIM_TESTS=ON
  #   colcon test --packages-select seed_rh8d_gazebo
  #
  # Each is runnable on its own too, e.g.
  #   python3 src/seed_rh8d_gazebo/test/test_sim_joint_limits.py mimic left
  option(SEED_SIM_TESTS
    "Also run the Gazebo integration tests (slow; needs ros_gz and gz_ros2_control)"
    OFF)
  if(SEED_SIM_TESTS)
    foreach(coupling mimic independent)
      foreach(side right left)
        ament_add_test(sim_driver_interface_${coupling}_${side}
          COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_driver_interface.py"
                  ${coupling} ${side}
          WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
          TIMEOUT 900
          GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        )
      endforeach()
    endforeach()

    foreach(side right left)
      ament_add_test(sim_joint_limits_mimic_${side}
        COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_joint_limits.py"
                mimic ${side}
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        TIMEOUT 1200
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      )
    endforeach()
    ament_add_test(sim_joint_limits_independent_right
      COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_joint_limits.py"
              independent right
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      TIMEOUT 1200
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )

    ament_add_test(sim_user_samples
      COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_user_samples.py"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      TIMEOUT 900
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )
  endif()
endif()

ament_package()
