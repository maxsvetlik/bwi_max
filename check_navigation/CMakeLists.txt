cmake_minimum_required(VERSION 2.8.3)
project(check_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib move_base_msgs
)

add_executable(checkNav src/nav.cpp)
target_link_libraries(
  checkNav
  ${catkin_LIBRARIES}
)
