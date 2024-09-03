cmake_minimum_required(VERSION 3.8)
project(iir_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_controllers
  diff_drive_controller
  ros2_control
  controller_manager
  backward_ros
  ament_cmake
  ament_index_cpp
  yaml_cpp_vendor
)



find_package(pluginlib REQUIRED) # Add this line



# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(diff_drive_controller SHARED
  src/diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(diff_drive_controller PUBLIC cxx_std_17)
target_include_directories(diff_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_drive_controller>
)
ament_target_dependencies(diff_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(diff_drive_controller PRIVATE "DIFF_DRIVE_CONTROLLER_BUILDING_DLL")


install(DIRECTORY urdf/ 
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(hardware_interface diff_drive_controller.xml)

ament_package()
