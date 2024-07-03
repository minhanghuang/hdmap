#ifndef HDMAP_SERVER_H_
#define HDMAP_SERVER_H_
#include <hdmap_common/util.h>
#include <hdmap_engine/common/log.h>
#include <hdmap_engine/common/param.h>
#include <hdmap_engine/engine.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <hdmap_msgs/msg/map.hpp>
#include <hdmap_msgs/msg/region.hpp>
#include <hdmap_msgs/msg/road.hpp>
#include <hdmap_msgs/msg/section.hpp>
#include <hdmap_msgs/srv/get_global_map.hpp>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hdmap {

class XMapServer : public rclcpp::Node {
 public:
  explicit XMapServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~XMapServer() = default;

  bool Init();

 private:
  bool Checkin();

  bool Hz(int rate);

  void SetupRosPublisher();

  void SetupRosSubscriptions();

  void SetupRosService();

  void SetupRosTimer();

  void PublishTimer();

  void ProcessCurrentRegionTimer();

  void PublishGlobalMapMarkers();

  void PublishCurrentRegion();

  void GlobalMapServiceCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Request> request,
      std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Response> response);

  void MousePositionCallback(
      const geometry_msgs::msg::PointStamped::SharedPtr msg);

  void GenerateGlobalMap();

  /**
   * @brief hdmap engine param ptr
   */
  Param::Ptr param_;

  /**
   * @brief ros timer
   */
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  /**
   * @brief timer counter
   */
  std::uint64_t rate_counter_ = 0;

  /**
   * @brief hdmap engine ptr
   */
  std::shared_ptr<Engine> engine_;

  /// ros sub
  // moues position
  std::mutex mouse_position_mutex_;
  const std::string mouse_position_topic_;
  std::queue<geometry_msgs::msg::PointStamped> mouse_position_msgs_q_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      mouse_position_sub_;

  /// ros pub
  // marker
  // const std::string merker_topic_;
  // visualization_msgs::msg::MarkerArray marker_array_msg_;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
  //     marker_pub_;

  // current region
  std::mutex current_region_mutex_;
  const std::string current_region_topic_;
  hdmap_msgs::msg::Region current_region_msg_;
  rclcpp::Publisher<hdmap_msgs::msg::Region>::SharedPtr current_region_pub_;

  /// ros service
  const std::string global_map_topic_;
  hdmap_msgs::msg::Map global_map_msg_;
  rclcpp::Service<hdmap_msgs::srv::GetGlobalMap>::SharedPtr global_map_srv_;
};

}  // namespace hdmap

#endif  // HDMAP_SERVER_H_
