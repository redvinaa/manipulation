#ifndef GEOMETRIC_POSE_ESTIMATION__UTILS_HPP_
#define GEOMETRIC_POSE_ESTIMATION__UTILS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace geometric_pose_estimation
{

class MeasureExecutionTime
{
public:
  MeasureExecutionTime(const std::string & name)
  : name_(name), start_time_(std::chrono::high_resolution_clock::now()) {}

  ~MeasureExecutionTime()
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
    RCLCPP_INFO(rclcpp::get_logger("geometric_pose_estimation"), "%s took %ld ms", name_.c_str(), duration);
  }

private:
  std::string name_;
  std::chrono::high_resolution_clock::time_point start_time_;
};

}

#endif  // GEOMETRIC_POSE_ESTIMATION__UTILS_HPP_
