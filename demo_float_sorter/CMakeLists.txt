cmake_minimum_required(VERSION 2.8.3)

## Declare ROS package name
project(demo_float_sorter)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package( ) 


## Link to ROS libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  demo_node
  demo_float_object
)


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${demo_node_INCLUDE_DIRS}
  ${demo_float_object_INCLUDE_DIRS}
)



## Declare a C++ executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES})


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${demo_node_LIBRARIES}
  ${demo_float_object_LIBRARIES}
)

