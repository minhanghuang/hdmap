#include "hdamp_server/server.h"

namespace hdmap {

XMapServer::XMapServer(const rclcpp::NodeOptions& options)
    : Node("hdmap_server_node", options),
      // merker_topic_("/hdmap_server/map_marker"),
      global_map_topic_("/hdmap_server/global_map"),
      send_map_topic_("/hdmap_server/send_map_file"),
      mouse_position_topic_("/hdmap_server/mouse_position"),
      current_region_topic_("/hdmap_server/current_region"),
      param_(std::make_shared<Param>()),
      engine_(std::make_shared<Engine>()) {
  this->declare_parameter<std::string>("map_path", "");
  param_->set_file_path(this->get_parameter("map_path").as_string());
  param_->set_step(0.1);
}

bool XMapServer::Init() {
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

  SetupRosPublisher();
  SetupRosSubscriptions();
  SetupRosService();
  SetupRosTimer();

  GenerateGlobalMap();
  return true;
}

bool XMapServer::Checkin() {
  if (nullptr == param_ || nullptr == engine_ || param_->file_path().empty()) {
    return false;
  }
  HDMAP_LOG_INFO("map file: %s", param_->file_path().c_str());
  // HDMAP_LOG_INFO("topic global map: %s", merker_topic_.c_str());
  return true;
}

bool XMapServer::Hz(int rate) {
  uint32_t mod = 100 / rate;
  return (rate_counter_ % mod) == 0 ? true : false;
}

void XMapServer::SetupRosPublisher() {
  // marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
  //     merker_topic_, 1);
  current_region_pub_ =
      this->create_publisher<hdmap_msgs::msg::Region>(current_region_topic_, 1);
}

void XMapServer::SetupRosSubscriptions() {
  mouse_position_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          mouse_position_topic_, 1,
          std::bind(&XMapServer::MousePositionCallback, this,
                    std::placeholders::_1));
}

void XMapServer::SetupRosService() {
  global_map_srv_ = this->create_service<hdmap_msgs::srv::GetGlobalMap>(
      global_map_topic_, std::bind(&XMapServer::GlobalMapServiceCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));
  send_map_srv_ = this->create_service<hdmap_msgs::srv::SendMapFile>(
      send_map_topic_, std::bind(&XMapServer::SendMapServiceCallback, this,
                                 std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));
}

void XMapServer::SetupRosTimer() {
  timers_.emplace_back(this->create_wall_timer(
      // 100Hz
      rclcpp::Rate(100).period(), std::bind(&XMapServer::PublishTimer, this)));
  timers_.emplace_back(this->create_wall_timer(
      // 10Hz
      rclcpp::Rate(10).period(),
      std::bind(&XMapServer::ProcessCurrentRegionTimer, this)));
}

void XMapServer::PublishTimer() {
  if (Hz(1)) {
    PublishGlobalMapMarkers();
  }
  if (Hz(10)) {
    PublishCurrentRegion();
  }
  rate_counter_++;
}

void XMapServer::ProcessCurrentRegionTimer() {
  geometry_msgs::msg::PointStamped mouse_position;
  {
    std::lock_guard<std::mutex> guard(mouse_position_mutex_);
    if (mouse_position_msgs_q_.empty()) {
      return;
    }
    mouse_position = mouse_position_msgs_q_.front();
    mouse_position_msgs_q_.pop();
  }
  auto search_results = engine_->GetNearestPoints(mouse_position.point.x,
                                                  mouse_position.point.y, 1);
  if (search_results.empty()) {
    return;
  }
  auto search_result = search_results.begin();

  // lane point
  geometry::Curve::Point point;
  if (!engine_->GetPointById(search_result->id, point)) {
    return;
  }

  // lane
  hdmap_msgs::msg::Lane lane_msg;
  geometry::Lane::ConstPtr lane = nullptr;
  const std::string lane_id = common::GetLaneIdByPointId(point.id());
  lane = engine_->GetLaneById(lane_id);
  if (nullptr != lane) {
    if (!lane->left_boundary().curve().pts().empty() &&
        !lane->right_boundary().curve().pts().empty()) {
      lane_msg.id = lane->id();
      // TODO: lane type
      lane_msg.lane_type = hdmap_msgs::msg::Lane::LANE_TYPE_STRAIGHT;
      const int line_size =
          common::MinValue(lane->central_curve().pts().size(),
                           lane->left_boundary().curve().pts().size(),
                           lane->right_boundary().curve().pts().size());
      for (int i = 0; i < line_size; i++) {
        hdmap_msgs::msg::Point point_msg;
        // // central curve point
        // point_msg.point.set__x(lane->central_curve().pts().at(i).x());
        // point_msg.point.set__y(lane->central_curve().pts().at(i).y());
        // point_msg.point.set__z(lane->central_curve().pts().at(i).z());
        // lane_msg.central_curve.pts.emplace_back(point_msg);

        // right boundary point
        point_msg.point.set__x(lane->left_boundary().curve().pts().at(i).x());
        point_msg.point.set__y(lane->left_boundary().curve().pts().at(i).y());
        point_msg.point.set__z(lane->left_boundary().curve().pts().at(i).z());
        lane_msg.left_boundary.pts.emplace_back(point_msg);

        // left boundary point
        point_msg.point.set__x(lane->right_boundary().curve().pts().at(i).x());
        point_msg.point.set__y(lane->right_boundary().curve().pts().at(i).y());
        point_msg.point.set__z(lane->right_boundary().curve().pts().at(i).z());
        lane_msg.right_boundary.pts.emplace_back(point_msg);
      }
    }
  }

  // fill msg
  {
    std::lock_guard<std::mutex> guard(current_region_mutex_);
    current_region_msg_.header.stamp = this->now();
    current_region_msg_.id = lane_msg.id;  // lane id
    current_region_msg_.distance = search_result->dist;
    current_region_msg_.point.x = point.x();
    current_region_msg_.point.y = point.y();
    current_region_msg_.heading = point.heading();
    current_region_msg_.s = point.start_position();
    current_region_msg_.lane = lane_msg;
  }
}

void XMapServer::PublishGlobalMapMarkers() {
  // marker_pub_->publish(marker_array_msg_);
}

void XMapServer::PublishCurrentRegion() {
  std::lock_guard<std::mutex> guard(current_region_mutex_);
  current_region_pub_->publish(current_region_msg_);
}

void XMapServer::GlobalMapServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Request> request,
    std::shared_ptr<hdmap_msgs::srv::GetGlobalMap::Response> response) {
  response->set__map(global_map_msg_);
}

void XMapServer::SendMapServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<hdmap_msgs::srv::SendMapFile::Request> request,
    std::shared_ptr<hdmap_msgs::srv::SendMapFile::Response> response) {
  // reload map(block)
  ReloadMap(request->map_file_info);
  response->set__map(global_map_msg_);
}

void XMapServer::MousePositionCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> guard(mouse_position_mutex_);
  if (!mouse_position_msgs_q_.empty()) {
    mouse_position_msgs_q_.pop();
  }
  mouse_position_msgs_q_.push(*msg);
}

void XMapServer::ReloadMap(const hdmap_msgs::msg::MapFileInfo& msg) {
  /// check in
  if (!hdmap::fs::exists(msg.file_path) ||
      hdmap_msgs::msg::MapFileInfo::MAP_TYPE_UNKNOWN == msg.map_type) {
    HDMAP_LOG_ERROR("%s file not found.", msg.file_path.c_str());
    return;
  }
  HDMAP_LOG_INFO("reload map file: %s", msg.file_path.c_str());
  param_->set_file_path(msg.file_path);
  param_->set_step(msg.resolution);
  engine_ = std::make_unique<Engine>();
  engine_->Init(*param_);
  GenerateGlobalMap();
  HDMAP_LOG_INFO("reload map done");
}

void XMapServer::GenerateGlobalMap() {
  // //// map marker
  // auto lanes = engine_->GetLanes();
  // for (int i = 0; i < lanes.size(); i++) {
  //   visualization_msgs::msg::Marker lane_msg;
  //   lane_msg.header.frame_id = "map";
  //   lane_msg.header.stamp = this->now();
  //   lane_msg.ns = "markers";
  //   lane_msg.id = i;
  //   lane_msg.action = visualization_msgs::msg::Marker::ADD;
  //   lane_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  //   lane_msg.pose.orientation.w = 1.0;
  //   lane_msg.scale.x = 0.1;
  //   lane_msg.color.r = 1.0;  // RGB
  //   lane_msg.color.a = 1.0;  // Alpha
  //   for (const auto& point : lanes.at(i)->central_curve().pts()) {
  //     geometry_msgs::msg::Point point_msg;
  //     point_msg.set__x(point.x());
  //     point_msg.set__y(point.y());
  //     point_msg.set__z(point.z());
  //     lane_msg.points.emplace_back(point_msg);
  //   }
  //   marker_array_msg_.markers.emplace_back(lane_msg);
  // }

  //// global map
  global_map_msg_.roads.clear();
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

          // left boundary point
          point_msg.point.set__x(lane->left_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(lane->left_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(lane->left_boundary().curve().pts().at(i).z());
          lane_msg.left_boundary.pts.emplace_back(point_msg);

          // right boundary point
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

          // left boundary point
          point_msg.point.set__x(lane->left_boundary().curve().pts().at(i).x());
          point_msg.point.set__y(lane->left_boundary().curve().pts().at(i).y());
          point_msg.point.set__z(lane->left_boundary().curve().pts().at(i).z());
          lane_msg.left_boundary.pts.emplace_back(point_msg);

          // right boundary point
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
    global_map_msg_.roads.emplace_back(road_msg);
  }
}

}  // namespace hdmap
