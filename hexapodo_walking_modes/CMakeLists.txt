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
  INCLUDE_DIRS include
  LIBRARIES tinyfsm
  CATKIN_DEPENDS roscpp std_msgs hexapodo_controller_interface
)


###########
## Build ##
###########

include_directories(
  include src
  ${catkin_INCLUDE_DIRS}
)

#add_library(altern_tripod_fem libraries/altern_tripod_fem/altern_tripod_fem.cpp)
add_library(tinyfsm libraries/tinyfsm/tinyfsm.cpp)

## Declare a C++ executable
#add_executable(tripode_alterno_joystick src/aleatorio_joystick_kinematic.cpp)
add_executable(tripode_alterno_me src/tripode_alterno_fsm.cpp)
#add_executable(todas_juntas src/modo_todas_juntas.cpp)

#add_dependencies(tripode_alterno_joystick ${catkin_EXPORTED_TARGETS})
add_dependencies(tripode_alterno_me ${catkin_EXPORTED_TARGETS})
#add_dependencies(todas_juntas ${catkin_EXPORTED_TARGETS})

#target_link_libraries(tripode_alterno_joystick ${catkin_LIBRARIES})
target_link_libraries(tripode_alterno_me tinyfsm ${catkin_LIBRARIES})
#target_link_libraries(todas_juntas ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS tinyfsm
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/tinyfsm
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
