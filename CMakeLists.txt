cmake_minimum_required(VERSION 2.8.3)
project(out2flume)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES out2flume
#  CATKIN_DEPENDS roscpp rosgraph_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rest src/rest.cpp)
target_link_libraries(rest ${catkin_LIBRARIES} curl)
target_link_libraries(rest ${catkin_LIBRARIES})
