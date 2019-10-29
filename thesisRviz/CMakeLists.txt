cmake_minimum_required(VERSION 2.8.3)
project(thesisRviz)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  demo_node
  roscpp
  visualization_msgs
  demo_location_object
  demo_detected_object
  demo_clock_library
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thesisRviz
CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 # include
  ${catkin_INCLUDE_DIRS}
  ${demo_node_INCLUDE_DIRS}
  ${demo_location_object_INCLUDE_DIRS}
  ${demo_detected_object_INCLUDE_DIRS}    
  ${demo_clock_library_INCLUDE_DIRS}  
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/thesisRviz.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
add_executable(${PROJECT_NAME}_node src/thesisRviz.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${demo_node_LIBRARIES}
   ${demo_location_object_LIBRARIES}
   ${demo_detected_object_LIBRARIES}
   ${demo_clock_library_LIBRARIES}
 )


