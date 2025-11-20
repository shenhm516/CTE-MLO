cmake_minimum_required(VERSION 2.8.3)
project(cte_mlo)

SET(CMAKE_BUILD_TYPE "Release")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(aarch64)|(AARCH64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 24)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=24)
    message("core for MP: 24")
  elseif(N GREATER 16)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=16)
    message("core for MP: 16")
  elseif(N GREATER 8)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=8)
    message("core for MP: 8")
  elseif(N GREATER 6)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=6)
    message("core for MP: 6")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  livox_ros_driver2
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}  
  include)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(cte_mlo src/laserMapping.cpp)
target_link_libraries(cte_mlo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} tbb)
target_include_directories(cte_mlo PRIVATE ${PYTHON_INCLUDE_DIRS})