cmake_minimum_required(VERSION 2.8.3)
project(macpcan_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
)
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    can_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
