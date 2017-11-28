cmake_minimum_required(VERSION 2.8.3)
project(hexapodo_description)

find_package(catkin REQUIRED COMPONENTS roslaunch urdf xacro) #robotnik_sensors)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch meshes urdf robots vrep
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
