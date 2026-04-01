cmake_minimum_required(VERSION 3.8)
project(motion_planning)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Force generation of compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Generate compile_commands.json" FORCE)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(bin_picking REQUIRED)
find_package(ifopt REQUIRED)
find_package(backward_ros REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  tf2_eigen
  geometry_msgs
  nav_msgs
  visualization_msgs
  interactive_markers
  bin_picking
  ifopt
  backward_ros
  angles
)

include_directories(include)

add_executable(2d_optimization src/2d_optimization.cpp)
target_link_libraries(2d_optimization ifopt::ifopt_ipopt ifopt::ifopt_core)
ament_target_dependencies(2d_optimization
  ${DEPENDENCIES}
)

add_executable(cspace_explorer src/cspace_explorer.cpp)
target_link_libraries(cspace_explorer ifopt::ifopt_ipopt ifopt::ifopt_core)
ament_target_dependencies(cspace_explorer
  ${DEPENDENCIES}
)

add_executable(simple_rrt_demo src/simple_rrt_demo.cpp)
target_link_libraries(simple_rrt_demo ${OpenCV_LIBS})
ament_target_dependencies(simple_rrt_demo rclcpp)

install(TARGETS 2d_optimization cspace_explorer simple_rrt_demo
  DESTINATION lib/${PROJECT_NAME}
)

# === Install ===
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
