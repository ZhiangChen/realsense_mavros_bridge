cmake_minimum_required(VERSION 2.8.3)
project(realsense_mavros_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

cs_add_library(bridge_lib src/bridge.cpp )


cs_add_executable(bridge_ros_node
        src/bridge_ros_node.cpp
)

target_link_libraries(bridge_ros_node bridge_lib)


##########
# EXPORT #
##########
cs_install()
cs_export()