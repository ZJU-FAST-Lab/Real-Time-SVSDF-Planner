cmake_minimum_required(VERSION 3.0.2)
project(plan_manager)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC  ")


## 查找 catkin 包和其他依赖
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  visualization_msgs
  gazebo_msgs
  plan_env
  robotstatemsgs
  utils
  swept_volume
  planner_algorithm
)


## 查找其他依赖包
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(tf REQUIRED)

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    robotstatemsgs 
    gazebo_ros 
    roscpp 
    std_msgs 
    visualization_msgs 
    plan_env 
    planner_algorithm
    utils
  DEPENDS EIGEN3 PCL
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GPERFTOOLS_INCLUDE_DIR}
)
include_directories("/usr/include/eigen3")
## 添加可执行文件
add_executable(global_planning 
  src/global_planning.cpp 
  src/plan_manager.cpp
)

## 链接库
target_link_libraries(global_planning
  ${catkin_LIBRARIES}
  ${GPERFTOOLS_LIB}
  ${GPERFTOOLS_TC_LIB}
)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed")