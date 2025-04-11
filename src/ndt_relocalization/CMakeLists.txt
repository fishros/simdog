cmake_minimum_required(VERSION 3.8)
project(ndt_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找PCL，不使用IMPORTED目标方式
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters search kdtree features registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找ndt_omp_ros2
find_package(ndt_omp_ros2 REQUIRED)

# Add executable
add_executable(ndt_relocalization_node 
  src/ndt_relocalization_node.cpp
)

# Include directories
target_include_directories(ndt_relocalization_node PRIVATE 
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Link libraries and dependencies
ament_target_dependencies(ndt_relocalization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

target_link_libraries(ndt_relocalization_node
  ${PCL_LIBRARIES}
  ndt_omp_ros2::ndt_omp
)

# Install
install(TARGETS
  ndt_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz configuration
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(map_publisher src/map_publisher.cpp)
target_include_directories(map_publisher PRIVATE
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(map_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(map_publisher
  ${PCL_LIBRARIES}
)

install(TARGETS map_publisher
  DESTINATION lib/${PROJECT_NAME}
)


# Optional: Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
