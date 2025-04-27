cmake_minimum_required(VERSION 3.0.2)
project(utils)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC  ")

# ## 设置编译选项
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC   -mavx -mfma -DNDEBUG")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
## 查找 catkin 包和其他依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  nodelet
  message_generation
  pcl_ros
  pcl_conversions
  robotstatemsgs
)

## 查找 Eigen3
find_package(Eigen3 REQUIRED)

## 声明消息文件
add_message_files(
  FILES
  debug.msg
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    visualization_msgs 
    nav_msgs 
    sensor_msgs 
    nodelet 
    message_runtime 
    pcl_ros 
    pcl_conversions 
    robotstatemsgs
  DEPENDS EIGEN3
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 添加库
add_library(${PROJECT_NAME}
  src/debug_publisher.cpp
)

## 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## 安装头文件
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## 安装库文件
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装消息文件
install(FILES debug.msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

## 如果有可执行文件，可以添加安装规则
# install(TARGETS nodelet_example
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
