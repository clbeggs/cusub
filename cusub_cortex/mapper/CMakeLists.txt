cmake_minimum_required(VERSION 2.8.3)
project(mapper)
set(Boost_DIR /home/epiphyte/boost_1_65_1/stage/lib)
set(Boost_INCLUDE_DIR /home/epiphyte/boost_1_65_1/boost)
set(Boost_LIBRARY_DIRS /home/epiphyte/boost_1_65_1/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  darknet_ros_msgs
  message_filters
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
message(STATUS "Found Boost version ${Boost_VERSION}" )

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    message_filters
    darknet_ros_msgs
    image_transport
    cv_bridge
)


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    # add_rostest_gtest(tests)
endif()
add_library(mapper
    src/mapper.cpp
    #src/particle.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include/
)

link_directories(${EIGEN3_LIBRARY_DIRS})


add_executable(mapper_node 
    src/mapper.cpp
    #src/particle.cpp
)

add_dependencies(mapper_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapper_node ${EIGEN3_LIBRARIES})
target_link_libraries(mapper_node ${catkin_LIBRARIES})


