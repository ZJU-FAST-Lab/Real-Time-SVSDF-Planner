
cmake_minimum_required(VERSION 3.0.2)
project(plan_env)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC  ")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  robotstatemsgs
  gazebo_ros
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  pcl_ros
)

find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plan_env
  CATKIN_DEPENDS robotstatemsgs gazebo_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_library( plan_env 
  src/sdf_map.cpp 
  src/raycast.cpp
)

target_link_libraries(plan_env 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

add_executable(exam_node 
  src/sdf_map_exam_node.cpp
)

target_link_libraries(exam_node 
  plan_env 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)



