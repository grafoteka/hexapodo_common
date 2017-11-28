cmake_minimum_required(VERSION 2.8.3)
project(hexapodo_walking_modes)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	roscpp
  std_msgs
  hexapodo_controller_interface
)



###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs hexapodo_controller_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tripode_alterno src/aleatorio_joystick.cpp)
add_executable(todas_juntas src/modo_todas_juntas.cpp)

add_dependencies(tripode_alterno ${catkin_EXPORTED_TARGETS})
add_dependencies(todas_juntas ${catkin_EXPORTED_TARGETS})

target_link_libraries(tripode_alterno ${catkin_LIBRARIES})
target_link_libraries(todas_juntas ${catkin_LIBRARIES})

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
