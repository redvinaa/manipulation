cmake_minimum_required(VERSION 3.8)
project(geometric_pose_estimation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(backward_ros REQUIRED)

find_package(OsqpEigen REQUIRED)

add_executable(icp_with_segmentation src/icp_with_segmentation.cpp)
target_include_directories(icp_with_segmentation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(icp_with_segmentation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(icp_with_segmentation OsqpEigen::OsqpEigen)
ament_target_dependencies(
  icp_with_segmentation
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "rviz_visual_tools"
  "tf2_eigen"
  "Eigen3"
  "backward_ros"
)

install(TARGETS icp_with_segmentation
  DESTINATION lib/${PROJECT_NAME})

# install directories
install(DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
