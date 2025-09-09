#ifndef BIN_PICKING__UTILS_HPP_
#define BIN_PICKING__UTILS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace bin_picking
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
    RCLCPP_INFO(rclcpp::get_logger("bin_picking"), "%s took %ld ms", name_.c_str(), duration);
  }

private:
  std::string name_;
  std::chrono::high_resolution_clock::time_point start_time_;
};

}  // namespace bin_picking

#endif  // BIN_PICKING__UTILS_HPP_
