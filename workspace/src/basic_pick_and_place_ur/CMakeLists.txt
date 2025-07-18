cmake_minimum_required(VERSION 3.8)
project(basic_pick_and_place_ur)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OsqpEigen REQUIRED)

add_executable(robot_painter src/robot_painter.cpp)
target_include_directories(robot_painter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(robot_painter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_painter
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "rviz_visual_tools"
  "tf2_eigen"
  "Eigen3"
)

add_executable(quadratic_programming src/quadratic_programming.cpp)
target_include_directories(quadratic_programming PUBLIC ${EIGEN3_INCLUDE_DIR})
target_compile_features(quadratic_programming PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(quadratic_programming OsqpEigen::OsqpEigen)

install(TARGETS robot_painter quadratic_programming
  DESTINATION lib/${PROJECT_NAME})

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
