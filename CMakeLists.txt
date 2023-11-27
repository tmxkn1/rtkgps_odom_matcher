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
    std_srvs
    message_runtime
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(Eigen3 REQUIRED)


####################
## Catkin Configs ##
####################

add_service_files(
  FILES
    SetPose.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS 
    include 
    include/libicp
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${${PROJECT_NAME}_CATKIN_DEPS})


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(libicp
    src/libicp/icp.cpp
    src/libicp/icpPointToPoint.cpp
    src/libicp/icpPointToPlane.cpp
    src/libicp/kdtree.cpp
    src/libicp/matrix.cpp
)

add_library(${PROJECT_NAME}
src/matcher.cpp
src/odomDataStore.cpp
)

add_executable(matcher_node src/matcher_node.cpp)

add_dependencies(${PROJECT_NAME} libicp ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} libicp ${catkin_LIBRARIES})
target_link_libraries(matcher_node libicp ${PROJECT_NAME} ${catkin_LIBRARIES})


###########
## Tests ##
###########

catkin_add_gtest(${PROJECT_NAME}-test tests/odomDataStore.test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
