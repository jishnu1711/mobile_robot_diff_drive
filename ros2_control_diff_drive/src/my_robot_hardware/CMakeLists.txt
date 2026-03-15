cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

# Treat all warnings as errors during development — catches mistakes early.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Find dependencies
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --------------------------------------------------------------------------
# Build the shared library
# --------------------------------------------------------------------------
# This produces libmy_robot_hardware.so — the name pluginlib looks for
# when it reads path="my_robot_hardware" in the plugin descriptor XML.
add_library(
  my_robot_hardware
  SHARED
  src/micro_ros_hardware.cpp
)

# Tell the compiler where to find our header.
target_include_directories(
  my_robot_hardware
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against all required ROS 2 libraries.
ament_target_dependencies(
  my_robot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  geometry_msgs
)

# --------------------------------------------------------------------------
# Register the plugin XML with ament
# --------------------------------------------------------------------------
# This is the critical step that makes pluginlib able to discover this
# plugin from any other package. It writes an entry into the ament
# resource index at install time.
pluginlib_export_plugin_description_file(
  hardware_interface
  my_robot_hardware.xml
)

# --------------------------------------------------------------------------
# Install targets
# --------------------------------------------------------------------------
# Install the compiled shared library into the standard lib directory.
install(
  TARGETS my_robot_hardware
  EXPORT my_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the public header so other packages could include it if needed.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the plugin descriptor XML itself.
install(
  FILES my_robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------------
# Ament package export
# --------------------------------------------------------------------------
ament_export_targets(my_robot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

ament_package()