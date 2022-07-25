cmake_minimum_required(VERSION 3.0.2)
project(vkistbot)

find_package(catkin REQUIRED)
catkin_package()
find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
)

catkin_package(
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_twist_keyboard src/omnidrive_velocity_controller.cpp)
target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})

add_executable(pdt_ndt_slam src/pso_ndt_slam.cpp)
target_link_libraries(pdt_ndt_slam  ${catkin_LIBRARIES})

add_executable(line_extraction src/line_extraction.cpp)
target_link_libraries(line_extraction  ${catkin_LIBRARIES})

