cmake_minimum_required(VERSION 3.16)
project(advrf_ros2_motors_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
foreach(dependency rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs builtin_interfaces advrf_ros2_interfaces std_srvs)
  find_package(${dependency} REQUIRED)
endforeach()


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
)

target_link_libraries(motors_controller_common motors_controller_math)
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

add_executable(converter_tx_motor_node src/nodes/converter_tx_motor_node.cpp)
target_link_libraries(converter_tx_motor_node motors_controller_common)
target_compile_options(converter_tx_motor_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(converter_tx_motor_node rclcpp sensor_msgs advrf_ros2_interfaces std_srvs)

add_executable(joint_goal_manager_node src/nodes/joint_goal_manager_node.cpp)
target_link_libraries(joint_goal_manager_node motors_controller_common)
target_compile_options(joint_goal_manager_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_goal_manager_node rclcpp rclcpp_action control_msgs sensor_msgs advrf_ros2_interfaces)

install(TARGETS joint_trajectory_controller_node converter_tx_motor_node joint_goal_manager_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS motors_controller_common motors_controller_math EXPORT export_motors_controller_common
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/test_joint_trajectory DESTINATION lib/${PROJECT_NAME})
install(FILES README.md plugin.xml DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_motors_controller_common HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs builtin_interfaces advrf_ros2_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_ecat_namespace
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ecat_namespace.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 15)
  ament_add_test(test_panel_config
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_panel_config.py"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_trajectory_generator
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_trajectory_generator.py"
      "$<TARGET_FILE:joint_trajectory_controller_node>" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_joint_trajectory" "$<TARGET_FILE:converter_tx_motor_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)



  ament_add_test(test_motor_panel
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_panel.py"
    ENV "ROS_DOMAIN_ID=218" "ROS_LOCALHOST_ONLY=1" "QT_QPA_PLATFORM=offscreen"
      "ROS_LOG_DIR=/tmp/advrf_panel_test_logs" "ROS_HOME=/tmp/advrf_panel_test_ros"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
  ament_add_test(test_action
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_action.py" "$<TARGET_FILE:joint_trajectory_controller_node>" "$<TARGET_FILE:converter_tx_motor_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 40)
  ament_add_test(test_joint_command
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_command.py" "$<TARGET_FILE:converter_tx_motor_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 25)
  ament_add_gtest(test_motion_math test/test_motion_math.cpp)
  target_link_libraries(test_motion_math motors_controller_common)
  ament_target_dependencies(test_motion_math trajectory_msgs rclcpp sensor_msgs advrf_ros2_interfaces)
  ament_add_test(test_go_to
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_go_to.py"
      "$<TARGET_FILE:joint_goal_manager_node>" "$<TARGET_FILE:joint_trajectory_controller_node>" "$<TARGET_FILE:converter_tx_motor_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 45)
  ament_add_gtest(test_interpolator test/test_interpolator.cpp)
  target_link_libraries(test_interpolator motors_controller_common)
  ament_target_dependencies(test_interpolator control_msgs rclcpp_action sensor_msgs advrf_ros2_interfaces)
endif()
ament_package()
