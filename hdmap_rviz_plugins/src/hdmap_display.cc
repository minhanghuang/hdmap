#include "hdmap_rviz_plugins/hdmap_display.h"

namespace hdmap_rviz_plugins {

MapDisplay::MapDisplay()
    : global_map_topic_("/hdmap_server/global_map"),
      send_map_topic_("/hdmap_server/send_map_file"),
      current_region_topic_("/hdmap_server/current_region"),
      mouse_position_topic_("/hdmap_server/mouse_position") {}

MapDisplay::~MapDisplay() {}

void MapDisplay::onInitialize() {
  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  current_region_ =
      std::make_shared<CurrentRegion>(scene_manager_, scene_node_);
  SetupRosSubscriptions();
  SetupRosPublisher();
  SetupRosService();
  SetupRosTimer();
  SetupOverlay();
  SetupRvizEvent();
  CallGlobalMap();
}

void MapDisplay::SetupRosSubscriptions() {
  current_region_sub_ = node_->create_subscription<hdmap_msgs::msg::Region>(
      current_region_topic_, 1,
      std::bind(&MapDisplay::CurrentRegionCallback, this,
                std::placeholders::_1));
}

void MapDisplay::SetupRosPublisher() {
  mouse_position_pub_ =
      node_->create_publisher<geometry_msgs::msg::PointStamped>(
          mouse_position_topic_, 1);
}

void MapDisplay::SetupRosService() {
  global_map_client_ =
      node_->create_client<hdmap_msgs::srv::GetGlobalMap>(global_map_topic_);
  send_map_client_ =
      node_->create_client<hdmap_msgs::srv::SendMapFile>(send_map_topic_);
  while (!global_map_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return;
    }
  }
}

void MapDisplay::SetupRosTimer() {
  timers_.emplace_back(node_->create_wall_timer(
      rclcpp::Rate(5).period(),
      std::bind(&MapDisplay::ShowCurrentRegion, this)));
}

void MapDisplay::SetupOverlay() {
  overlay_ = std::make_shared<OverlayComponent>("current_region");
  overlap_ui_ = std::make_shared<CurrentRegionOverlayUI>();
  overlay_->SetPosition(10, 10, HorizontalAlignment::LEFT,
                        VerticalAlignment::TOP);
}

void MapDisplay::SetupRvizEvent() {
  EventManager::GetInstance()->RegisterCallback(
      EventManager::EventType::kMouseCursorEvent,
      std::bind(&MapDisplay::HandleEventFromMouseCursor, this,
                std::placeholders::_1));
  EventManager::GetInstance()->RegisterCallback(
      EventManager::EventType::kSelectFileEvent,
      std::bind(&MapDisplay::HandleEventFromSelectFile, this,
                std::placeholders::_1));
}

void MapDisplay::CallGlobalMap() {
  std::lock_guard<std::mutex> guard(mutex_);
  auto request = std::make_shared<hdmap_msgs::srv::GetGlobalMap::Request>();
  auto response_callback =
      [this](
          rclcpp::Client<hdmap_msgs::srv::GetGlobalMap>::SharedFuture future) {
        auto response = future.get();
        GlobalMapMsgToBillboardLines(response->map, rviz_lines_);
      };
  auto future =
      global_map_client_->async_send_request(request, response_callback);
}

void MapDisplay::CallSendMap(
    const hdmap_msgs::msg::MapFileInfo& map_file_info) {
  std::lock_guard<std::mutex> guard(mutex_);
  auto request = std::make_shared<hdmap_msgs::srv::SendMapFile::Request>();
  request->map_file_info = map_file_info;
  auto response_callback =
      [this](
          rclcpp::Client<hdmap_msgs::srv::SendMapFile>::SharedFuture future) {
        auto response = future.get();
        GlobalMapMsgToBillboardLines(response->map, rviz_lines_);
      };
  auto future =
      send_map_client_->async_send_request(request, response_callback);
}

void MapDisplay::ShowCurrentRegion() {
  hdmap_msgs::msg::Region current_region;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (current_region_msg_.id.empty()) {
      return;
    }
    current_region = current_region_msg_;
  }

  /// overlay text
  *overlap_ui_->mutable_id() = current_region.id;
  overlap_ui_->mutable_point()->clear();
  overlap_ui_->mutable_point()->emplace_back(current_region.point.x);
  overlap_ui_->mutable_point()->emplace_back(current_region.point.y);
  overlap_ui_->mutable_point()->emplace_back(current_region.heading);
  overlay_->Clean();
  overlay_->Update(overlap_ui_.get());
  overlay_->Show();

  /// current lane
  current_region_->mutable_boundary()->clear();
  if (!current_region.lane.left_boundary.pts.empty() &&
      !current_region.lane.right_boundary.pts.empty()) {
    const int line_size = std::max<int>(
        0, std::min<int>(current_region.lane.left_boundary.pts.size(),
                         current_region.lane.right_boundary.pts.size()));
    if (line_size > 0) {
      current_region_->mutable_boundary()->setMaxPointsPerLine(line_size * 2 +
                                                               1);
      // left boundary pts
      for (int i = 0; i < line_size; i++) {
        current_region_->mutable_boundary()->addPoint(
            Ogre::Vector3(current_region.lane.left_boundary.pts.at(i).point.x,
                          current_region.lane.left_boundary.pts.at(i).point.y,
                          current_region.lane.left_boundary.pts.at(i).point.z));
      }
      // right boundary pts
      for (int i = line_size - 1; i >= 0; i--) {
        current_region_->mutable_boundary()->addPoint(Ogre::Vector3(
            current_region.lane.right_boundary.pts.at(i).point.x,
            current_region.lane.right_boundary.pts.at(i).point.y,
            current_region.lane.right_boundary.pts.at(i).point.z));
      }
      // left first point
      current_region_->mutable_boundary()->addPoint(Ogre::Vector3(
          current_region.lane.left_boundary.pts.begin()->point.x,
          current_region.lane.left_boundary.pts.begin()->point.y,
          current_region.lane.left_boundary.pts.begin()->point.z));
    }
  }
}

void MapDisplay::CurrentRegionCallback(
    const hdmap_msgs::msg::Region::SharedPtr msg) {
  std::lock_guard<std::mutex> guard(mutex_);
  current_region_msg_ = *msg;
}

void MapDisplay::HandleEventFromMouseCursor(void* msg) {
  geometry_msgs::msg::PointStamped* raw_msg =
      static_cast<geometry_msgs::msg::PointStamped*>(msg);
  mouse_position_pub_->publish(*raw_msg);
}

void MapDisplay::HandleEventFromSelectFile(void* msg) {
  hdmap_msgs::msg::MapFileInfo* raw_msg =
      static_cast<hdmap_msgs::msg::MapFileInfo*>(msg);
  this->CallSendMap(*raw_msg);
  
}

void MapDisplay::GlobalMapMsgToBillboardLines(
    const hdmap_msgs::msg::Map& map,
    std::vector<std::shared_ptr<rviz_rendering::BillboardLine>>& lines) {
  if (map.roads.empty()) {
    return;
  }
  lines.clear();
  for (const auto& road : map.roads) {
    for (const auto& section : road.sections) {
      for (const auto& lane : section.lanes) {
        const int line_size = std::max<int>(
            0, std::min<int>(lane.central_curve.pts.size(),
                             std::min<int>(lane.left_boundary.pts.size(),
                                           lane.left_boundary.pts.size())));
        if (0 == line_size) {
          continue;
        }
        {
          /// fill central line
          auto rviz_line = std::make_shared<rviz_rendering::BillboardLine>(
              scene_manager_, scene_node_);
          rviz_line->setMaxPointsPerLine(line_size);
          rviz_line->setNumLines(1);
          for (int i = 0; i < line_size; i++) {
            rviz_line->addPoint(
                Ogre::Vector3(lane.central_curve.pts.at(i).point.x,
                              lane.central_curve.pts.at(i).point.y,
                              lane.central_curve.pts.at(i).point.z));
          }
          rviz_lines_.emplace_back(rviz_line);
        }
        {
          /// fill left boundary line
          auto rviz_line = std::make_shared<rviz_rendering::BillboardLine>(
              scene_manager_, scene_node_);
          rviz_line->setMaxPointsPerLine(line_size);
          rviz_line->setNumLines(1);
          for (int i = 0; i < line_size; i++) {
            rviz_line->addPoint(
                Ogre::Vector3(lane.left_boundary.pts.at(i).point.x,
                              lane.left_boundary.pts.at(i).point.y,
                              lane.left_boundary.pts.at(i).point.z));
          }
          rviz_lines_.emplace_back(rviz_line);
        }
        {
          /// fill right boundary line
          auto rviz_line = std::make_shared<rviz_rendering::BillboardLine>(
              scene_manager_, scene_node_);
          rviz_line->setMaxPointsPerLine(line_size);
          rviz_line->setNumLines(1);
          for (int i = 0; i < line_size; i++) {
            rviz_line->addPoint(
                Ogre::Vector3(lane.right_boundary.pts.at(i).point.x,
                              lane.right_boundary.pts.at(i).point.y,
                              lane.right_boundary.pts.at(i).point.z));
          }
          rviz_lines_.emplace_back(rviz_line);
        }
      }
    }
  }
}

}  // namespace hdmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hdmap_rviz_plugins::MapDisplay, rviz_common::Display)
