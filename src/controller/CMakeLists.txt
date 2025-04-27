cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC  ")
project(controller)

## 找到 catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robotstatemsgs
  message_generation
  utils
)

find_package(Eigen3 REQUIRED)

## 添加消息文件
# 如果有自己的消息文件，添加到这里
# add_message_files(
#   FILES
# )


## 生成消息
# generate_messages(
#   DEPENDENCIES
# )

## 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs 
  robotstatemsgs
  utils
)

## 包含目录
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


## 添加可执行文件
add_executable(controller_node src/controller_node.cpp src/controller.cpp)

## 添加依赖关系
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库
target_link_libraries(controller_node
  ${catkin_LIBRARIES}
)
