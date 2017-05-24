cmake_minimum_required(VERSION 2.8.3)
project(competition_solution)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
  keyboard
  tf
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)

find_package(Boost REQUIRED COMPONENTS thread)

#todo: add dependencies
catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(solution_node src/solution_node.cpp)
target_link_libraries(solution_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
##install(PROGRAMS scripts/ariac_example_node.py
##  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##)

## Mark executables and/or libraries for installation
install(TARGETS solution_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
