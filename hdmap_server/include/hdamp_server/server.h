#ifndef HDMAP_SERVER_H_
#define HDMAP_SERVER_H_
#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hdmap_engine/common/log.h"
#include "hdmap_engine/common/param.h"
#include "hdmap_engine/engine.h"
#include "hdmap_msgs/msg/map.hpp"
#include "hdmap_msgs/msg/road.hpp"
#include "hdmap_msgs/msg/section.hpp"
#include "hdmap_msgs/srv/get_global_map.hpp"

using namespace std::chrono_literals;

namespace hdmap {

class HDMapServer : public rclcpp::Node {
 public:
  explicit HDMapServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HDMapServer() = default;

  bool Init();

 private:
  void TimerCallback();

  bool Checkin();

  bool Hz(int rate);

  void GenerateGlobalMap();

  void GlobalMapServiceCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Request> request,
      std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Response> response);

  /**
   * @brief hdmap engine param ptr
   */
  Param::Ptr param_;

  /**
   * @brief ros timer 100Hz
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief timer counter
   */
  std::uint64_t rate_counter_ = 0;

  /**
   * @brief hdmap engine ptr
   */
  std::shared_ptr<Engine> engine_;

  /// ros publisher
  const std::string merker_topic_;
  std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_array_msg_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  /// ros service
  const std::string global_map_topic_;
  std::shared_ptr<hdmap_msgs::msg::Map> global_map_msg_;
  rclcpp::Service<hdmap_msgs::srv::GetGlobalMap>::SharedPtr global_map_srv_;
};

}  // namespace hdmap

#endif  // HDMAP_SERVER_H_
