cmake_minimum_required(VERSION 3.0.2)
project(pc_visualization)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED
    roscpp
)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(pc_visualization src/pc_visualization.cpp)
target_link_libraries(pc_visualization
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
