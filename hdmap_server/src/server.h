#ifndef HDMAP_SERVER_H_
#define HDMAP_SERVER_H_
#include <array>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hdmap_engine/common/log.h"
#include "hdmap_engine/common/param.h"
#include "hdmap_engine/engine.h"

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

  /**
   * @brief global map publisher
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      global_map_pub_;
  std::string global_map_topic_;
  std::shared_ptr<visualization_msgs::msg::MarkerArray> global_map_;
};

}  // namespace hdmap

#endif  // HDMAP_SERVER_H_
