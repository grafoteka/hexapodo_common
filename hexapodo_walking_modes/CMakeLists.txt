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
  LIBRARIES hexapod_common_methods stand_by altern_tripod
  CATKIN_DEPENDS roscpp std_msgs hexapodo_controller_interface
)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

###########
## Build ##
###########

include_directories(
  include src
  ${catkin_INCLUDE_DIRS}
)

#add_library(altern_tripod_fem libraries/altern_tripod_fem/altern_tripod_fem.cpp)
add_library(common_methods libraries/common_methods/hexapod_common_methods.cpp)
add_library(stand_by libraries/stand_by/stand_by.cpp)
add_library(altern_tripod libraries/altern_tripod/altern_tripod.cpp)

## Declare a C++ executable
#add_executable(tripode_alterno_joystick src/aleatorio_joystick_kinematic.cpp)
add_executable(hexapodo_fsm src/hexapodo_fsm.cpp)
add_executable(hexapodo_fsm_listener src/hexapodo_fsm_listener.cpp)
#add_executable(todas_juntas src/modo_todas_juntas.cpp)

#add_dependencies(tripode_alterno_joystick ${catkin_EXPORTED_TARGETS})
#add_dependencies(hexapodo_fsm stand_by altern_tripod ${catkin_EXPORTED_TARGETS})
add_dependencies(hexapodo_fsm ${catkin_EXPORTED_TARGETS})
add_dependencies(hexapodo_fsm_listener stand_by altern_tripod ${catkin_EXPORTED_TARGETS})
#add_dependencies(todas_juntas ${catkin_EXPORTED_TARGETS})

#target_link_libraries(tripode_alterno_joystick ${catkin_LIBRARIES})
target_link_libraries(hexapodo_fsm ${catkin_LIBRARIES})
#target_link_libraries(hexapodo_fsm stand_by altern_tripod ${catkin_LIBRARIES})
target_link_libraries(hexapodo_fsm_listener stand_by altern_tripod ${catkin_LIBRARIES})
target_link_libraries(stand_by common_methods ${catkin_LIBRARIES})
target_link_libraries(altern_tripod common_methods ${catkin_LIBRARIES})

#target_link_libraries(altern_tripod common_methods ${catkin_LIBRARIES})
#target_link_libraries(stand_by common_methods ${catkin_LIBRARIES})
#target_link_libraries(common_methods ${catkin_LIBRARIES})
#target_link_libraries(todas_juntas ${catkin_LIBRARIES})



