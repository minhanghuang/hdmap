#include "hdmap_rviz_plugins/hdmap_display.h"

namespace hdmap_rviz_plugins {

MapDisplay::MapDisplay()
    : global_map_topic_("/hdmap_server/global_map"),
      send_map_topic_("/hdmap_server/send_map_file"),
      current_region_topic_("/hdmap_server/current_region"),
      mouse_position_topic_("/hdmap_server/mouse_position"),
      file_loaded_(false) {}

MapDisplay::~MapDisplay() {}

void MapDisplay::onInitialize() {
  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  scene_manager_ = context_->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
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
  current_region_sub_ = nh_->create_subscription<hdmap_msgs::msg::Region>(
      current_region_topic_, 1,
      std::bind(&MapDisplay::CurrentRegionCallback, this,
                std::placeholders::_1));
}

void MapDisplay::SetupRosPublisher() {
  mouse_position_pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(
      mouse_position_topic_, 1);
}

void MapDisplay::SetupRosService() {
  global_map_client_ =
      nh_->create_client<hdmap_msgs::srv::GetGlobalMap>(global_map_topic_);
  send_map_client_ =
      nh_->create_client<hdmap_msgs::srv::SendMapFile>(send_map_topic_);
  while (!global_map_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return;
    }
  }
}

void MapDisplay::SetupRosTimer() {
  timers_.emplace_back(
      nh_->create_wall_timer(rclcpp::Rate(5).period(),
                             std::bind(&MapDisplay::ShowCurrentRegion, this)));
}

void MapDisplay::SetupOverlay() {
  current_region_overlay_ =
      std::make_shared<OverlayComponent<CurrentRegionOverlayUI>>(
          "current_region");
  current_region_overlay_->SetPosition(10, 10, HorizontalAlignment::LEFT,
                                       VerticalAlignment::TOP);

  mouse_position_overlay_ =
      std::make_shared<OverlayComponent<MousePositionOverlayUI>>(
          "mouse_position");
  mouse_position_overlay_->SetPosition(0, 25, HorizontalAlignment::LEFT,
                                       VerticalAlignment::BOTTOM);
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
        ConvertToBillboardLines(response->map);
      };
  auto future =
      global_map_client_->async_send_request(request, response_callback);
}

void MapDisplay::CallSendMap(
    const hdmap_msgs::msg::MapFileInfo& map_file_info) {
  {
    hdmap::common::WriteLock(map_file_info_mutex_);
    map_file_info_ = map_file_info;
  }
  auto request = std::make_shared<hdmap_msgs::srv::SendMapFile::Request>();
  request->map_file_info = map_file_info;
  auto response_callback =
      [this](
          rclcpp::Client<hdmap_msgs::srv::SendMapFile>::SharedFuture future) {
        auto response = future.get();
        if (!response->map.roads.empty()) {
          ConvertToBillboardLines(response->map);
          file_loaded_ = true;
        } else {
          file_loaded_ = false;
        }
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
  auto current_region_overlap_ui = current_region_overlay_->ui();
  {
    hdmap::common::ReadLock(map_file_info_mutex_);
    *current_region_overlap_ui->mutable_file_path() = map_file_info_.file_path;
  }
  *current_region_overlap_ui->mutable_id() = current_region.id;
  current_region_overlap_ui->mutable_point()->clear();
  current_region_overlap_ui->mutable_point()->emplace_back(
      current_region.point.x);
  current_region_overlap_ui->mutable_point()->emplace_back(
      current_region.point.y);
  current_region_overlap_ui->mutable_point()->emplace_back(
      current_region.heading);
  current_region_overlay_->Clean();
  current_region_overlay_->Show();

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
  if (file_loaded_) {
    // pub mouse position
    mouse_position_pub_->publish(*raw_msg);
    // show mouse position
    auto mouse_position_overlap_ui = mouse_position_overlay_->ui();
    mouse_position_overlap_ui->set_x(raw_msg->point.x);
    mouse_position_overlap_ui->set_y(raw_msg->point.y);
    mouse_position_overlap_ui->set_z(raw_msg->point.z);
    mouse_position_overlay_->Clean();
    mouse_position_overlay_->Show();
  }
}

void MapDisplay::HandleEventFromSelectFile(void* msg) {
  hdmap_msgs::msg::MapFileInfo* raw_msg =
      static_cast<hdmap_msgs::msg::MapFileInfo*>(msg);
  this->CallSendMap(*raw_msg);
}

void MapDisplay::ConvertToBillboardLines(const hdmap_msgs::msg::Map& msg) {
  if (msg.roads.empty()) {
    return;
  }

  if (!rviz_lines_.empty()) {
    // destroy lines
    for (auto& line : rviz_lines_) {
      line->clear();
    }
    rviz_lines_.clear();
  }

  for (const auto& road : msg.roads) {
    for (const auto& section : road.sections) {
      for (const auto& lane : section.lanes) {
        const int line_size = std::max<int>(
            0, std::min<int>(lane.central_curve.pts.size(),
                             std::min<int>(lane.left_boundary.pts.size(),
                                           lane.left_boundary.pts.size())));
        if (0 == line_size) {
          continue;
        }
        auto rviz_line =
            std::make_shared<MapDisplay::RvizLine>(scene_manager_, scene_node_);
        rviz_line->setMaxPointsPerLine(line_size);
        rviz_line->setNumLines(3);

        /// fill central line
        for (int i = 0; i < line_size; i++) {
          rviz_line->addPoint(
              Ogre::Vector3(lane.central_curve.pts.at(i).point.x,
                            lane.central_curve.pts.at(i).point.y,
                            lane.central_curve.pts.at(i).point.z));
        }
        rviz_line->finishLine();

        /// fill left boundary line
        for (int i = 0; i < line_size; i++) {
          rviz_line->addPoint(
              Ogre::Vector3(lane.left_boundary.pts.at(i).point.x,
                            lane.left_boundary.pts.at(i).point.y,
                            lane.left_boundary.pts.at(i).point.z));
        }
        rviz_line->finishLine();

        /// fill right boundary line
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

  context_->queueRender();
}

}  // namespace hdmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hdmap_rviz_plugins::MapDisplay, rviz_common::Display)
