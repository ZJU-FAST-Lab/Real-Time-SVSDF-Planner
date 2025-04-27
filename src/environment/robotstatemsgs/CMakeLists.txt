cmake_minimum_required(VERSION 3.0.2)
project(robotstatemsgs)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC  ")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  robotState.msg
  robotTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# find_package(cmake_modules REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_generation std_msgs geometry_msgs
)

