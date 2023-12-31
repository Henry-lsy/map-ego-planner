cmake_minimum_required(VERSION 2.8.3)

project(global_planner)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  # visualization_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES global_planner
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
#  DEPENDS system_lib
)

add_executable(global_planner_node 
  src/global_planner_node.cpp
  src/global_planner.cpp)

target_link_libraries(global_planner_node
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(global_planner src/global_planner.cpp)

target_link_libraries(global_planner INTERFACE
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)  
