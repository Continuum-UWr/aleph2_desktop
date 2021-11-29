cmake_minimum_required(VERSION 3.5)
project(aleph2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

# aleph2_gazebo_differential_plugin
add_library(aleph2_gazebo_differential_plugin SHARED
  src/differential_plugin.cpp
)
ament_target_dependencies(aleph2_gazebo_differential_plugin
  gazebo_dev
  gazebo_ros
)

# aleph2_gazebo_hardware_plugins
add_library(aleph2_gazebo_hardware_plugins SHARED
  src/aleph2_gazebo_system.cpp
)
ament_target_dependencies(aleph2_gazebo_hardware_plugins
  gazebo_dev
  hardware_interface
  rclcpp
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    aleph2_gazebo_differential_plugin
    aleph2_gazebo_hardware_plugins
  LIBRARY DESTINATION lib
)

ament_export_libraries(
  aleph2_gazebo_differential_plugin
  aleph2_gazebo_hardware_plugins
)

pluginlib_export_plugin_description_file(gazebo_ros2_control gazebo_hardware_plugins.xml)

ament_package()
