cmake_minimum_required(VERSION 3.5)
project(ndt_omp_ros2)

# -mavx causes a lot of errors!!
#add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
#set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  pcl_ros
#)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_omp
#)

###########
## Build ##
###########
include_directories(
  include
)

# 将ndt_omp库设置为SHARED库以便导出
add_library(ndt_omp SHARED
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

target_include_directories(ndt_omp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ndt_omp
  ${PCL_LIBRARIES}
)

add_executable(align
  apps/align.cpp
)
add_dependencies(align
  ndt_omp
)
target_link_libraries(align
  ${PCL_LIBRARIES}
  ndt_omp
)

ament_target_dependencies(align
  rclcpp
  std_msgs
)

############
## INSTALL ##
############

# 安装可执行文件
install(TARGETS 
  align
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件和导出目标
install(
  TARGETS ndt_omp
  EXPORT ndt_omp_ros2-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(
  DIRECTORY "include/"
  DESTINATION include
)

# 导出目标
ament_export_targets(ndt_omp_ros2-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(PCL)
ament_export_include_directories(include)
ament_export_libraries(ndt_omp)

ament_package()
