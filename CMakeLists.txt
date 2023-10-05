cmake_minimum_required(VERSION 3.1)
project(rtkgps_odom_matcher)


set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    nav_msgs
    geometry_msgs
    nmea_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME}
    src/matcher.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(matcher_node src/matcher_node.cpp)
target_link_libraries(matcher_node ${PROJECT_NAME} ${catkin_LIBRARIES})
