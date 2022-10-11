cmake_minimum_required(VERSION 3.5)
project(aleph2_gz)

find_package(ament_cmake REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_library(aleph2_gz_differential_system SHARED
  src/differential_system.cpp
)
set_property(TARGET aleph2_gz_differential_system PROPERTY CXX_STANDARD 17)
target_link_libraries(aleph2_gz_differential_system
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
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
