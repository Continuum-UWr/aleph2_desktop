 
cmake_minimum_required(VERSION 3.0.2)

project(aleph2_viz)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch rviz)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)