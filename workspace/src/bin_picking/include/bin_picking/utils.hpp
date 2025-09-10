#ifndef BIN_PICKING__UTILS_HPP_
#define BIN_PICKING__UTILS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace bin_picking
{

class MeasureExecutionTimeScoped
{
public:
  MeasureExecutionTimeScoped(const std::string & name)
  : name_(name), start_time_(std::chrono::high_resolution_clock::now()) {}

  ~MeasureExecutionTimeScoped()
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
    RCLCPP_INFO(rclcpp::get_logger("bin_picking"), "%s took %ld ms", name_.c_str(), duration);
  }

private:
  std::string name_;
  std::chrono::high_resolution_clock::time_point start_time_;
};

class MeasureExecutionTime
{
public:
  MeasureExecutionTime(const std::string & name = "Execution", bool auto_start = true)
  : name_(name)
  {
    if (auto_start) {
      start();
    }
  }

  inline void start()
  {
    if (start_time_) {
      RCLCPP_WARN(rclcpp::get_logger("bin_picking"), "Timer already started, overwriting");
    }

    start_time_ = std::make_shared<std::chrono::high_resolution_clock::time_point>(
      std::chrono::high_resolution_clock::now());
  }

  inline void stop(const std::string & name = "")
  {
    if (!start_time_) {
      RCLCPP_WARN(rclcpp::get_logger("bin_picking"), "Timer was not started!");
      return;
    }

    const std::string current_name = name.empty() ? name_ : name;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - *start_time_).count();
    RCLCPP_INFO(rclcpp::get_logger("bin_picking"), "%s took %ld ms", current_name.c_str(), duration);

    start_time_.reset();
  }

private:
  std::string name_;
  std::shared_ptr<std::chrono::high_resolution_clock::time_point> start_time_;
};

}  // namespace bin_picking

#endif  // BIN_PICKING__UTILS_HPP_
