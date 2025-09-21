cmake_minimum_required(VERSION 3.8)
project(bin_picking)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Force generation of compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Generate compile_commands.json" FORCE)

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
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(DEPENDENCIES
  rclcpp
  tf2_ros
  sensor_msgs
  pcl_conversions
  # pcl_ros
  rviz_visual_tools
  tf2_eigen
  moveit_ros
  moveit_ros_planning_interface
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(find_grasp_pose_node src/find_grasp_pose.cpp)
target_link_libraries(find_grasp_pose_node ${PCL_LIBRARIES})
ament_target_dependencies(find_grasp_pose_node
  ${DEPENDENCIES}
)
install(TARGETS find_grasp_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Install ===
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
