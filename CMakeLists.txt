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

catkin_package(
  INCLUDE_DIRS include include/libicp
  LIBRARIES ${PROJECT_NAME})


find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
    src/matcher.cpp
    src/odomDataStore.cpp
    # src/odomDataStoreD.cpp
    src/libicp/icp.cpp
    src/libicp/icpPointToPoint.cpp
    src/libicp/icpPointToPlane.cpp
    src/libicp/kdtree.cpp
    src/libicp/matrix.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(${PROJECT_NAME}-test src/odomDataStore.test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

add_executable(matcher_node src/matcher_node.cpp)
target_link_libraries(matcher_node ${PROJECT_NAME} ${catkin_LIBRARIES})
