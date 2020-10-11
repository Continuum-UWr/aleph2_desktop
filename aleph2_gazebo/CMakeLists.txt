cmake_minimum_required(VERSION 3.0.2)
project(aleph2_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY 
  config
  launch
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
