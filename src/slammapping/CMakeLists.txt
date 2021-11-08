cmake_minimum_required(VERSION 3.0.2)
project(slammapping)

set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  fsd_msgs
  fsd_common_msgs
  visualization_msgs
  fsd_tools
  )
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros
  DEPENDS
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)


# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  #src/DemoSlamMapping_withoutgsp.cpp
  src/Fortest_DemoSlamMapping_withoutgsp.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  )
