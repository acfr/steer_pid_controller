cmake_minimum_required(VERSION 3.16)
project(steer_pid_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
  control_toolbox
  angles
  visualization_msgs
  ros2_controllers_interfaces
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

generate_parameter_library(steer_pid_controller_parameters
  src/steer_pid_controller_parameter.yaml
)

add_library(
  steer_pid_controller
  SHARED
  src/steer_pid_controller.cpp
)
target_compile_features(steer_pid_controller PUBLIC cxx_std_17)
target_include_directories(steer_pid_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/steer_pid_controller>")
target_link_libraries(steer_pid_controller PUBLIC
  steer_pid_controller_parameters)
ament_target_dependencies(steer_pid_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(steer_pid_controller PRIVATE "STEER_PID_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface steer_pid_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/steer_pid_controller
)

install(
  TARGETS steer_pid_controller steer_pid_controller_parameters
  EXPORT export_steer_pid_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_steer_pid_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
