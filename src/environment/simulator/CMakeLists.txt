cmake_minimum_required(VERSION 3.0.2)
project(simulator)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC  ")

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  robotstatemsgs
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)


find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES simulator
 CATKIN_DEPENDS robotstatemsgs geometry_msgs roscpp std_msgs visualization_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_executable(simulator src/simulator.cpp)

target_link_libraries(simulator
  ${catkin_LIBRARIES}
)

add_executable(global_map_node
  src/global_map.cpp
)

target_link_libraries( 
  global_map_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
