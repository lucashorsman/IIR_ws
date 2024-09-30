set(_AMENT_PACKAGE_NAME "iir_base")
set(iir_base_VERSION "0.0.0")
set(iir_base_MAINTAINER "riplab <lucashorsman@gmail.com>")
set(iir_base_BUILD_DEPENDS "backward_ros" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "rclpy" "controller_manager" "diagnostic_updater" "ros2_control" "controller_interface" "diff_drive_controller")
set(iir_base_BUILDTOOL_DEPENDS "ament_cmake_python")
set(iir_base_BUILD_EXPORT_DEPENDS "backward_ros" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "rclpy" "controller_manager" "diagnostic_updater" "ros2_control" "controller_interface" "diff_drive_controller")
set(iir_base_BUILDTOOL_EXPORT_DEPENDS )
set(iir_base_EXEC_DEPENDS "joint_state_broadcaster" "joint_state_publisher_gui" "robot_state_publisher" "ros2_controllers_test_nodes" "ros2controlcli" "ros2launch" "rviz2" "xacro" "backward_ros" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle" "rclpy" "controller_manager" "diagnostic_updater" "ros2_control" "controller_interface" "diff_drive_controller")
set(iir_base_TEST_DEPENDS "ament_cmake_pytest" "launch_testing_ros" "liburdfdom-tools" "xacro" "ament_lint_auto" "ament_lint_common")
set(iir_base_GROUP_DEPENDS )
set(iir_base_MEMBER_OF_GROUPS )
set(iir_base_DEPRECATED "")
set(iir_base_EXPORT_TAGS)
list(APPEND iir_base_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
