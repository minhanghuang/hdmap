#include "hdamp_server/server.h"

#include <climits>

namespace hdmap {

HDMapServer::HDMapServer(const rclcpp::NodeOptions& options)
    : Node("hdmap_server_node", options),
      merker_topic_("/hdmap_server/map_marker"),
      global_map_topic_("/hdmap_server/global_map"),
      param_(std::make_shared<Param>()),
      engine_(std::make_shared<Engine>()) {
  this->declare_parameter("map_file_path", "");
  param_->set_file_path(this->get_parameter("map_file_path").as_string());
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
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      merker_topic_, 1);

  // create server
  global_map_srv_ = this->create_service<hdmap_msgs::srv::GetGlobalMap>(
      global_map_topic_, std::bind(&HDMapServer::GlobalMapServiceCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));

  GenerateGlobalMap();

  // create timer
  timer_ = this->create_wall_timer(
      10ms /*100Hz*/, std::bind(&HDMapServer::TimerCallback, this));

  return true;
}

void HDMapServer::TimerCallback() {
  if (Hz(1)) {
    // marker_pub_->publish(*marker_array_msg_);
  }
  rate_counter_++;
}

bool HDMapServer::Checkin() {
  if (nullptr == param_ || nullptr == engine_ || param_->file_path().empty()) {
    return false;
  }
  HDMAP_LOG_INFO("map file: %s", param_->file_path().c_str());
  HDMAP_LOG_INFO("topic global map: %s", merker_topic_.c_str());
  return true;
}

bool HDMapServer::Hz(int rate) {
  uint32_t mod = 100 / rate;
  return (rate_counter_ % mod) == 0 ? true : false;
}

void HDMapServer::GenerateGlobalMap() {
  marker_array_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  global_map_msg_ = std::make_shared<hdmap_msgs::msg::Map>();

  //// map marker
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
    marker_array_msg_->markers.emplace_back(lane_msg);
  }

  //// global map
  auto map = engine_->GetRoads();
  for (const auto& road : map) {
    hdmap_msgs::msg::Road road_msg;
    for (const auto& section : road->sections()) {
      hdmap_msgs::msg::Section section_msg;
      // left lanes
      for (const auto& lane : section->left_lanes()) {
        hdmap_msgs::msg::Lane lane_msg;
        int line_size = std::max<int>(
            0, std::min<int>(
                   lane->central_curve().pts().size(),
                   std::min<int>(lane->left_boundary().curve().pts().size(),
                                 lane->right_boundary().curve().pts().size())));
        for (int i = 0; i < line_size; i++) {
          hdmap_msgs::msg::Point point_msg;

          // central curve point
          point_msg.point.set__x(lane->central_curve().pts().at(i).x());
          point_msg.point.set__y(lane->central_curve().pts().at(i).y());
          point_msg.point.set__z(lane->central_curve().pts().at(i).z());
          lane_msg.central_curve.pts.emplace_back(point_msg);

          // right boundary point
          point_msg.point.set__x(lane->left_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(lane->left_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(lane->left_boundary().curve().pts().at(i).z());
          lane_msg.left_boundary.pts.emplace_back(point_msg);

          // left boundary point
          point_msg.point.set__x(
              lane->right_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(
              lane->right_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(
              lane->right_boundary().curve().pts().at(i).z());
          lane_msg.right_boundary.pts.emplace_back(point_msg);
        }
        section_msg.lanes.emplace_back(lane_msg);
      }

      // right lanes
      for (const auto& lane : section->right_lanes()) {
        hdmap_msgs::msg::Lane lane_msg;
        int line_size = std::max<int>(
            0, std::min<int>(
                   lane->central_curve().pts().size(),
                   std::min<int>(lane->left_boundary().curve().pts().size(),
                                 lane->right_boundary().curve().pts().size())));
        for (int i = 0; i < line_size; i++) {
          hdmap_msgs::msg::Point point_msg;

          // central curve point
          point_msg.point.set__x(lane->central_curve().pts().at(i).x());
          point_msg.point.set__y(lane->central_curve().pts().at(i).y());
          point_msg.point.set__z(lane->central_curve().pts().at(i).z());
          lane_msg.central_curve.pts.emplace_back(point_msg);

          // right boundary point
          point_msg.point.set__x(lane->left_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(lane->left_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(lane->left_boundary().curve().pts().at(i).z());
          lane_msg.left_boundary.pts.emplace_back(point_msg);

          // left boundary point
          point_msg.point.set__x(
              lane->right_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(
              lane->right_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(
              lane->right_boundary().curve().pts().at(i).z());
          lane_msg.right_boundary.pts.emplace_back(point_msg);
        }
        section_msg.lanes.emplace_back(lane_msg);
      }

      // center lane
      {
        int line_size = std::max<int>(
            0,
            std::min<int>(section->center_lane()->central_curve().pts().size(),
                          std::min<int>(section->center_lane()
                                            ->left_boundary()
                                            .curve()
                                            .pts()
                                            .size(),
                                        section->center_lane()
                                            ->right_boundary()
                                            .curve()
                                            .pts()
                                            .size())));
        hdmap_msgs::msg::Lane lane_msg;
        for (int i = 0; i < line_size; i++) {
          hdmap_msgs::msg::Point point_msg;

          // central curve point
          point_msg.point.set__x(
              section->center_lane()->central_curve().pts().at(i).x());
          point_msg.point.set__y(
              section->center_lane()->central_curve().pts().at(i).y());
          point_msg.point.set__z(
              section->center_lane()->central_curve().pts().at(i).z());
          lane_msg.central_curve.pts.emplace_back(point_msg);

          point_msg.point.set__x(
              section->center_lane()->left_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(
              section->center_lane()->left_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(
              section->center_lane()->left_boundary().curve().pts().at(i).z());
          lane_msg.left_boundary.pts.emplace_back(point_msg);

          point_msg.point.set__x(
              section->center_lane()->right_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(
              section->center_lane()->right_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(
              section->center_lane()->right_boundary().curve().pts().at(i).z());
          lane_msg.right_boundary.pts.emplace_back(point_msg);
        }
        section_msg.lanes.emplace_back(lane_msg);
      }  // center lane
      road_msg.sections.emplace_back(section_msg);
    }
    global_map_msg_->roads.emplace_back(road_msg);
  }
}

void HDMapServer::GlobalMapServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Request> request,
    std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Response> response) {
  std::cout << "GlobalMapServiceCallback1" << std::endl;
  response->set__map(*global_map_msg_);
  std::cout << "GlobalMapServiceCallback2" << std::endl;
}

}  // namespace hdmap
