#include "hdmap_rviz_plugins/select_file_tool.h"

namespace hdmap_rviz_plugins {

SelectFileTool::SelectFileTool() : rviz_common::Tool() {}

SelectFileTool::~SelectFileTool() {}

void SelectFileTool::onInitialize() {
  Tool::onInitialize();
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  SetupOverlay();
}

void SelectFileTool::activate() { this->ProcessButton(); }

void SelectFileTool::deactivate() {}

int SelectFileTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  hdmap::common::WriteLock(mutex_);
  Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();
  Ogre::Viewport* viewport = camera->getViewport();
  float screen_x = static_cast<float>(event.x) / viewport->getActualWidth();
  float screen_y = static_cast<float>(event.y) / viewport->getActualHeight();
  Ogre::Ray ray = camera->getCameraToViewportRay(screen_x, screen_y);
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0);
  std::pair<bool, Ogre::Real> result = ray.intersects(ground_plane);
  if (result.first) {
    Ogre::Vector3 intersection_point = ray.getPoint(result.second);
    /// show
    overlap_ui_->set_x(intersection_point.x);
    overlap_ui_->set_y(intersection_point.y);
    overlap_ui_->set_z(intersection_point.z);
    overlay_->Clean();
    overlay_->Update(overlap_ui_.get());
    overlay_->Show();

    mouse_position_msgs_.header.stamp = node_->get_clock()->now();
    mouse_position_msgs_.point.set__x(intersection_point.x);
    mouse_position_msgs_.point.set__y(intersection_point.y);
    mouse_position_msgs_.point.set__z(intersection_point.z);
    EventManager::GetInstance()->TriggerEvent(
        EventManager::EventType::kMouseCursorEvent, &mouse_position_msgs_);
  }

  // 3D view can be rotated using the mouse
  if (event.panel->getViewController()) {
    event.panel->getViewController()->handleMouseEvent(event);
    setCursor(event.panel->getViewController()->getCursor());
  }
  return 0;
}

void SelectFileTool::SetupOverlay() {
  overlay_ = std::make_shared<OverlayComponent>("mouse_position");
  overlap_ui_ = std::make_shared<MousePositionOverlayUI>();
  overlay_->SetPosition(0, 25, HorizontalAlignment::LEFT,
                        VerticalAlignment::BOTTOM);
}

bool SelectFileTool::ProcessButton() {
  const QString file_path =
      QFileDialog::getOpenFileName(nullptr, tr("Select File"), "",
                                   tr("OpenDRIVE/XML Files (*.xodr *.xml);;"
                                      "OpenDRIVE Files (*.xodr);;"
                                      "XML Files (*.xml)"));
  if (!hdmap::fs::exists(file_path.toStdString())) {
    return false;
  }
  map_file_info_msg_.header.stamp = node_->get_clock()->now();
  map_file_info_msg_.uuid = hdmap::common::GenerateUuid();
  map_file_info_msg_.file_path = file_path.toStdString();
  map_file_info_msg_.map_type =
      hdmap_msgs::msg::MapFileInfo::MAP_TYPE_OPENDRIVE;
  EventManager::GetInstance()->TriggerEvent(
      EventManager::EventType::kSelectFileEvent, &map_file_info_msg_);
  return true;
}

}  // namespace hdmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hdmap_rviz_plugins::SelectFileTool, rviz_common::Tool)
