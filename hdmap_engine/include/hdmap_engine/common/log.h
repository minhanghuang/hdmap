#ifndef HDMAP_COMMON_LOG_H_
#define HDMAP_COMMON_LOG_H_

#include <rclcpp/rclcpp.hpp>

namespace hdmap {

#define HDMAP_LOG_DEBUG(...) \
  RCLCPP_DEBUG(rclcpp::get_logger("hdmap"), __VA_ARGS__)

#define HDMAP_LOG_INFO(...) \
  RCLCPP_INFO(rclcpp::get_logger("hdmap"), __VA_ARGS__)

#define HDMAP_LOG_WARN(...) \
  RCLCPP_WARN(rclcpp::get_logger("hdmap"), __VA_ARGS__)

#define HDMAP_LOG_ERROR(...) \
  RCLCPP_ERROR(rclcpp::get_logger("hdmap"), __VA_ARGS__)

}  // namespace hdmap

#endif  // HDMAP_COMMON_LOG_H_
