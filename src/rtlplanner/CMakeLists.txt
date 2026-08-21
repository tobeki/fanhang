cmake_minimum_required(VERSION 3.0.2)
project(rtlplanner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mars_quadrotor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs mars_quadrotor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rtl_planner
  src/rtl_planner.cpp
)

target_link_libraries(rtl_planner
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
