cmake_minimum_required(VERSION 3.5)
project(aleph2_gz)

find_package(ament_cmake REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

add_library(aleph2_gz_differential_system SHARED
  src/differential_system.cpp
)

target_link_libraries(aleph2_gz_differential_system
  gz-sim::core
  gz-plugin::core
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    aleph2_gz_differential_system
  DESTINATION lib
)

ament_package()
