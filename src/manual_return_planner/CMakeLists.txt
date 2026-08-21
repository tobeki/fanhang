cmake_minimum_required(VERSION 3.0.2)
project(manual_return_planner)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  mars_quadrotor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manual_return_core
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs visualization_msgs std_msgs std_srvs pcl_ros pcl_conversions mars_quadrotor_msgs
  DEPENDS Eigen3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(manual_return_core src/manual_return_planner.cpp)
target_link_libraries(manual_return_core ${catkin_LIBRARIES})

add_executable(manual_return_node src/manual_return_node.cpp)
target_link_libraries(manual_return_node manual_return_core ${catkin_LIBRARIES})

add_executable(command_gate_node src/command_gate_node.cpp)
target_link_libraries(command_gate_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_manual_return test/test_manual_return.cpp)
  if(TARGET test_manual_return)
    target_link_libraries(test_manual_return manual_return_core gtest_main ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_command_gate test/test_command_gate.cpp)
  if(TARGET test_command_gate)
    target_link_libraries(test_command_gate manual_return_core gtest_main ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_metrics test/test_metrics.cpp)
  if(TARGET test_metrics)
    target_link_libraries(test_metrics manual_return_core gtest_main ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS manual_return_core manual_return_node command_gate_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/analyze_return_runs.py scripts/analyze_tracking_runs.py scripts/check_benchmark_runs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
