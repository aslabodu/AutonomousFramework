cmake_minimum_required(VERSION 2.8.3)

## Declare ROS package name
project(thesisDemoEnv3)


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
  parent_comm
  wheelsMessage
  commandMessage
  demo_location_object
  demo_detected_object
  demo_clock_library
)


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${demo_node_INCLUDE_DIRS}
  ${wheelsMessage_INCLUDE_DIRS}
  ${commandMessage_INCLUDE_DIRS}
  ${parent_comm_INCLUDE_DIRS}  
  ${demo_location_object_INCLUDE_DIRS}
  ${demo_detected_object_INCLUDE_DIRS}    
  ${demo_clock_library_INCLUDE_DIRS}  
)


## directories for other necessary libraries
link_directories(
	lib/tinyxml2
)


## Declare a C++ executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES})


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${demo_node_LIBRARIES}
  ${wheelsMessage_LIBRARIES}
  ${commandMessage_LIBRARIES}
  ${parent_comm_LIBRARIES}
  ${demo_location_object_LIBRARIES}
  ${demo_detected_object_LIBRARIES}
  ${demo_clock_library_LIBRARIES}
  tinyxml2
)

