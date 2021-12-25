cmake_minimum_required(VERSION 3.5)
project(aleph2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# aleph2_gazebo_differential_plugin
add_library(aleph2_gazebo_differential_plugin SHARED
  src/differential_plugin.cpp
)
ament_target_dependencies(aleph2_gazebo_differential_plugin
  gazebo_dev
  gazebo_ros
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    aleph2_gazebo_differential_plugin
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
