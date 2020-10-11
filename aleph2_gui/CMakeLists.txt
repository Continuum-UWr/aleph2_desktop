cmake_minimum_required(VERSION 3.0.2)
project(aleph2_gui)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(
  DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES rqt_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
