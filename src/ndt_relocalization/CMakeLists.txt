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
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

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
  pcl_filters
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS
  ndt_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
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
