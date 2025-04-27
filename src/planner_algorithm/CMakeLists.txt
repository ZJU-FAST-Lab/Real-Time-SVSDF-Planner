cmake_minimum_required(VERSION 3.0.2)
project(planner_algorithm)

# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 --std=c++14 -fPIC -fopenmp   -mavx -mfma -DNDEBUG"  )
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC   -fopenmp")


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
  plan_env
  swept_volume
)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)



include_directories(${EIGEN3_INCLUDE_DIR})

add_library(lmbm SHARED IMPORTED)
set_target_properties(lmbm PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../utils/include/utils/lmbm.so)
catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES ${PROJECT_NAME} planner_algorithm
   CATKIN_DEPENDS  
   roscpp 
   rospy 
   std_msgs 
   sensor_msgs 
   visualization_msgs 
   message_generation 
   utils
   plan_env
   swept_volume
#   DEPENDS system_lib
)

file(GLOB myheaders CONFIGURE_DEPENDS  "${CMAKE_SOURCE_DIR}/planner_algorithm/include/planner_algorithm/*.hpp") 

add_library(planner_algorithm
  src/front_end_astar.cpp
  src/BSpline_optimizer.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
)

target_link_libraries(planner_algorithm PUBLIC
  OpenMP::OpenMP_CXX
  lmbm
)


