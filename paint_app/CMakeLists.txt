cmake_minimum_required(VERSION 2.8.3)
project(paint_app)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
				 std_msgs 
				 roscpp 
				 genmsg
				 moveit_ros_planning_interface 
				 industrial_trajectory_filters 
				 )

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include ${catkin_INCLUDE_DIRS}
										${industrial_trajectory_filters_INCLUDE_DIRS})
										
add_executable(listener_paint src/listener.cpp)
target_link_libraries(listener_paint ${catkin_LIBRARIES})







