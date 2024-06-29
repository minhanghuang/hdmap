#include "hdmap_rviz_plugins/mouse_tool.h"

namespace hdmap_rviz_plugins {

MouseTool::MouseTool()
    : rviz_common::Tool(),
      mouse_position_topic_("/hdmap_server/mouse_position") {}

MouseTool::~MouseTool() {}

void MouseTool::onInitialize() {
  Tool::onInitialize();
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  mouse_position_pub_ =
      node_->create_publisher<geometry_msgs::msg::PointStamped>(
          mouse_position_topic_, 1);
  SetupOverlay();
}

void MouseTool::activate() {}

void MouseTool::deactivate() {}

int MouseTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  std::lock_guard<std::mutex> guard(mutex_);
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

    /// pub
    mouse_position_msgs_.header.stamp = node_->get_clock()->now();
    mouse_position_msgs_.point.set__x(intersection_point.x);
    mouse_position_msgs_.point.set__y(intersection_point.y);
    mouse_position_msgs_.point.set__z(intersection_point.z);
    mouse_position_pub_->publish(mouse_position_msgs_);
  }

  // 3D view can be rotated using the mouse
  if (event.panel->getViewController()) {
    event.panel->getViewController()->handleMouseEvent(event);
    setCursor(event.panel->getViewController()->getCursor());
  }
  return 0;
}

void MouseTool::SetupOverlay() {
  overlay_ = std::make_shared<OverlayComponent>("mouse_position");
  overlap_ui_ = std::make_shared<MousePositionOverlayUI>();
  overlay_->SetPosition(0, 25, HorizontalAlignment::LEFT,
                        VerticalAlignment::BOTTOM);
}

}  // namespace hdmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hdmap_rviz_plugins::MouseTool, rviz_common::Tool)
