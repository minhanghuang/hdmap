#ifndef HDMAP_COMMON_LOG_H_
#define HDMAP_COMMON_LOG_H_

#include <rclcpp/rclcpp.hpp>

namespace hdmap {

#define HDMAP_LOGGER rclcpp::get_logger("hdmap")

#define HDMAP_LOG_DEBUG(...) RCLCPP_DEBUG(HDMAP_LOGGER, __VA_ARGS__)

#define HDMAP_LOG_INFO(...) RCLCPP_INFO(HDMAP_LOGGER, __VA_ARGS__)

#define HDMAP_LOG_WARN(...) RCLCPP_WARN(HDMAP_LOGGER, __VA_ARGS__)

#define HDMAP_LOG_ERROR(...) RCLCPP_ERROR(HDMAP_LOGGER, __VA_ARGS__)

}  // namespace hdmap

#endif  // HDMAP_ENGINE_COMMON_LOG_H_
