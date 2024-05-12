#include "server.h"

namespace hdmap {

HDMapServer::HDMapServer(const rclcpp::NodeOptions& options)
    : Node("hdmap_server_node", options),
      param_(std::make_shared<Param>()),
      engine_(std::make_shared<Engine>()) {
  // declare this node's parameters
  this->declare_parameter("map_file_path", "");
  this->declare_parameter("topic_global_map", "");
  param_->set_file_path(this->get_parameter("map_file_path").as_string());
  global_map_topic_ = this->get_parameter("topic_global_map").as_string();
}

bool HDMapServer::Init() {
  // check in
  if (!Checkin()) {
    HDMAP_LOG_ERROR("hdmap server check in exception");
    return false;
  }

  // create engine
  if (!engine_->Init(*param_)) {
    HDMAP_LOG_ERROR("engine load map file exception");
    return false;
  }

  // create publisher
  global_map_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          global_map_topic_, 1);

  GenerateGlobalMap();

  // create timer
  timer_ = this->create_wall_timer(
      10ms /*100Hz*/, std::bind(&HDMapServer::TimerCallback, this));

  return true;
}

void HDMapServer::TimerCallback() {
  if (Hz(1)) {
    global_map_pub_->publish(*global_map_);
  }
  rate_counter_++;
}

bool HDMapServer::Checkin() {
  if (nullptr == param_ || nullptr == engine_ || param_->file_path().empty() ||
      global_map_topic_.empty()) {
    return false;
  }
  HDMAP_LOG_INFO("map file: %s", param_->file_path().c_str());
  HDMAP_LOG_INFO("topic global map: %s", global_map_topic_.c_str());
  return true;
}

bool HDMapServer::Hz(int rate) {
  uint32_t mod = 100 / rate;
  return (rate_counter_ % mod) == 0 ? true : false;
}

void HDMapServer::GenerateGlobalMap() {
  if (nullptr == global_map_) {
    global_map_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  }
  auto lanes = engine_->GetLanes();
  for (int i = 0; i < lanes.size(); i++) {
    visualization_msgs::msg::Marker lane_msg;
    lane_msg.header.frame_id = "map";
    lane_msg.header.stamp = this->now();
    lane_msg.ns = "markers";
    lane_msg.id = i;
    lane_msg.action = visualization_msgs::msg::Marker::ADD;
    lane_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lane_msg.pose.orientation.w = 1.0;
    lane_msg.scale.x = 0.1;  // 线段宽度
    lane_msg.color.r = 1.0;  // RGB
    lane_msg.color.a = 1.0;  // 透明度
    for (const auto& point : lanes.at(i)->central_curve().pts()) {
      geometry_msgs::msg::Point point_msg;
      point_msg.set__x(point.x());
      point_msg.set__y(point.y());
      point_msg.set__z(point.z());
      lane_msg.points.emplace_back(point_msg);
    }
    global_map_->markers.emplace_back(lane_msg);
  }
}

}  // namespace hdmap
