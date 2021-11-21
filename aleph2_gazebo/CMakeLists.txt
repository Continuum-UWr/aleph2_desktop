cmake_minimum_required(VERSION 3.5)
project(aleph2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(aleph2_gazebo_differential_plugin SHARED
  src/differential_plugin.cpp
)
ament_target_dependencies(aleph2_gazebo_differential_plugin
  gazebo_dev
  gazebo_ros
)
ament_export_libraries(aleph2_gazebo_differential_plugin)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS aleph2_gazebo_differential_plugin
  LIBRARY DESTINATION lib
)

ament_package()
