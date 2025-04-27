cmake_minimum_required(VERSION 3.0.2)
project(swept_volume)
# set(CMAKE_BUILD_TYPE "Release")
# # set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC   -mavx -mfma -DNDEBUG"  )
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC  ")


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  message_generation
  utils
  pcl_ros
  pcl_conversions
  plan_env
)
include_directories(${EIGEN3_INCLUDE_DIR})


catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS  
   roscpp 
   rospy 
   std_msgs 
   sensor_msgs 
   visualization_msgs 
   message_generation 
   utils
   pcl_ros
    pcl_conversions
   plan_env
#   DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/swept_volume_manager.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
)