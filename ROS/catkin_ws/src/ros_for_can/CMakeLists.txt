cmake_minimum_required(VERSION 2.8.3)
project(ros_for_can)

find_package(catkin REQUIRED COMPONENTS
  libpcan
  roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_for_can
  CATKIN_DEPENDS libpcan roscpp
  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
	include/ros_for_can
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ros_for_can
	src/ros_for_can.cpp
	src/IniFile.cpp
)

## Declare a cpp executable
# add_executable(ros_for_can_node src/ros_for_can.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ros_for_can_node ros_for_can_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ros_for_can_node
#   ${catkin_LIBRARIES}
# )

add_executable(ros_for_can_receive src/receive.cpp)
add_executable(ros_for_can_write src/write.cpp)
target_link_libraries(ros_for_can_receive ros_for_can ${catkin_LIBRARIES})
target_link_libraries(ros_for_can_write ros_for_can ${catkin_LIBRARIES})
