cmake_minimum_required(VERSION 3.8)
project(bin_picking)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features)
# find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rviz_visual_tools REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# === Library (Composable Node) ===
add_library(find_grasp_pose SHARED
  src/find_grasp_pose.cpp
)

target_include_directories(find_grasp_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(find_grasp_pose
  rclcpp
  rclcpp_components
  tf2_ros
  sensor_msgs
  # pcl_ros
  pcl_conversions
  rviz_visual_tools
  tf2_eigen
)

target_link_libraries(find_grasp_pose
  ${PCL_LIBRARIES}
)

# Register composable node
rclcpp_components_register_nodes(find_grasp_pose "FindGraspPose")

# === Install ===
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS find_grasp_pose
  EXPORT export_find_grasp_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
