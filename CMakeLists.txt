cmake_minimum_required(VERSION 3.16)
project(advrf_ros2_motors_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
foreach(dependency rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs builtin_interfaces advrf_ros2_interfaces std_srvs)
  find_package(${dependency} REQUIRED)
endforeach()

find_package(yaml-cpp REQUIRED)

add_library(motors_controller_math src/math/spline.cpp)
target_compile_features(motors_controller_math PUBLIC cxx_std_17)
target_compile_options(motors_controller_math PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motors_controller_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(motors_controller_common
  src/lib/trajectory_utils.cpp
  src/lib/motor_types.cpp
  src/lib/controller_config.cpp
  src/lib/go_to.cpp
  src/lib/repl_cmd_manager.cpp
)

target_link_libraries(motors_controller_common yaml-cpp motors_controller_math)
target_compile_features(motors_controller_common PUBLIC cxx_std_17)
target_compile_options(motors_controller_common PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motors_controller_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motors_controller_common rclcpp sensor_msgs advrf_ros2_interfaces trajectory_msgs builtin_interfaces)

add_executable(joint_trajectory_controller_node src/nodes/joint_trajectory_controller_node.cpp)
target_link_libraries(joint_trajectory_controller_node motors_controller_common)
target_compile_options(joint_trajectory_controller_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_trajectory_controller_node rclcpp rclcpp_action control_msgs sensor_msgs advrf_ros2_interfaces std_srvs)

add_executable(joint_command_controller_node src/nodes/joint_command_controller_node.cpp)
target_link_libraries(joint_command_controller_node motors_controller_common)
target_compile_options(joint_command_controller_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_command_controller_node rclcpp sensor_msgs advrf_ros2_interfaces std_srvs)

add_executable(joint_go_to_node src/nodes/joint_go_to_node.cpp)
target_link_libraries(joint_go_to_node motors_controller_common)
target_compile_options(joint_go_to_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_go_to_node rclcpp rclcpp_action control_msgs sensor_msgs advrf_ros2_interfaces)

add_executable(motor_power_node src/nodes/motor_power_node.cpp)
target_link_libraries(motor_power_node motors_controller_common)
target_compile_options(motor_power_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(motor_power_node rclcpp std_srvs advrf_ros2_interfaces)

install(TARGETS motor_power_node joint_trajectory_controller_node joint_command_controller_node joint_go_to_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS motors_controller_common motors_controller_math EXPORT export_motors_controller_common
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/test_joint_trajectory DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md plugin.xml DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_motors_controller_common HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs builtin_interfaces advrf_ros2_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_launch_config
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_config.py"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_trajectory_generator
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_trajectory_generator.py"
      "$<TARGET_FILE:joint_trajectory_controller_node>" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_joint_trajectory" "$<TARGET_FILE:joint_command_controller_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
  ament_add_test(test_motor_profiles
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_profiles.py"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_motor_panel
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_panel.py"
    ENV "ROS_DOMAIN_ID=218" "ROS_LOCALHOST_ONLY=1" "QT_QPA_PLATFORM=offscreen"
      "ROS_LOG_DIR=/tmp/advrf_panel_test_logs" "ROS_HOME=/tmp/advrf_panel_test_ros"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
  ament_add_test(test_motor_power
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_power.py"
      "$<TARGET_FILE:motor_power_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
  ament_add_test(test_action
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_action.py" "$<TARGET_FILE:joint_trajectory_controller_node>" "$<TARGET_FILE:joint_command_controller_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 40)
  ament_add_test(test_joint_command
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_command.py" "$<TARGET_FILE:joint_command_controller_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 25)
  ament_add_gtest(test_motion_math test/test_motion_math.cpp)
  target_link_libraries(test_motion_math motors_controller_common)
  ament_target_dependencies(test_motion_math trajectory_msgs rclcpp sensor_msgs advrf_ros2_interfaces)
  ament_add_test(test_go_to
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_go_to.py"
      "$<TARGET_FILE:joint_go_to_node>" "$<TARGET_FILE:joint_trajectory_controller_node>" "$<TARGET_FILE:joint_command_controller_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 45)
  ament_add_gtest(test_repl_cmd_manager test/test_repl_cmd_manager.cpp)
  target_link_libraries(test_repl_cmd_manager motors_controller_common)
  ament_target_dependencies(test_repl_cmd_manager rclcpp advrf_ros2_interfaces)
  set_tests_properties(test_repl_cmd_manager PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=191;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/advrf_repl_test_logs")
  ament_add_gtest(test_interpolator test/test_interpolator.cpp)
  target_link_libraries(test_interpolator motors_controller_common)
  ament_target_dependencies(test_interpolator control_msgs rclcpp_action sensor_msgs advrf_ros2_interfaces)
endif()
ament_package()
