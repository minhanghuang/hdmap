#include "hdmap_rviz_plugins/hdmap_rviz_plugins.h"

namespace hdmap_rviz_plugins {

GlobalMapDisplay::GlobalMapDisplay()
    : global_map_topic_("/hdmap_server/global_map") {}

GlobalMapDisplay::~GlobalMapDisplay() {}

void GlobalMapDisplay::onInitialize() {
  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  global_map_client_ =
      node_->create_client<hdmap_msgs::srv::GetGlobalMap>(global_map_topic_);
  while (!global_map_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return;
    }
  }

  ShowGlobalMap();
}

void GlobalMapDisplay::ShowGlobalMap() {
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

void GlobalMapDisplay::GlobalMapMsgToBillboardLines(
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
PLUGINLIB_EXPORT_CLASS(hdmap_rviz_plugins::GlobalMapDisplay,
                       rviz_common::Display)
