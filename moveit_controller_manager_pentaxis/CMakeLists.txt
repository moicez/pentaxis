cmake_minimum_required(VERSION 2.8.3)
project(moveit_controller_manager_pentaxis)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  roscpp
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(include)

add_library(moveit_controller_manager_pentaxis src/moveit_controller_manager_pentaxis.cpp)
target_link_libraries(moveit_controller_manager_pentaxis ${catkin_LIBRARIES} ${Boost_LIBRARIES} libGalil.so)

# This is not a working controller, so we do not install it. However, the lines below
# should be enabled for functional controllers

#install(TARGETS moveit_controller_manager_example LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(FILES moveit_controller_manager_pentaxis_plugin_description.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#       )
