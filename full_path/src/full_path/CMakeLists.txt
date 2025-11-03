cmake_minimum_required(VERSION 3.0.2)
project(full_path)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  turtlesim
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES full_path
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
${YAML_CPP_INCLUDE_DIRS}
)

# c++
add_executable(${PROJECT_NAME}_node src/ccpp.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_executable(pub_path src/pub_path.cpp)
target_link_libraries(pub_path
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_executable(purepursuit src/purepursuit.cpp)
target_link_libraries(purepursuit
  ${catkin_LIBRARIES}
)

