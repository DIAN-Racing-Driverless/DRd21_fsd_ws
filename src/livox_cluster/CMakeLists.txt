cmake_minimum_required(VERSION 2.8.3)
project(livox_cluster)
SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  message_filters
  )
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros message_filters
  DEPENDS
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# include_directories( /usr/include/eigen3 )

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/livox_cluster.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
