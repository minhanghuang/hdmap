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
}

void MouseTool::activate() {}

void MouseTool::deactivate() {}

int MouseTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  std::lock_guard<std::mutex> guard(mutex_);
  mouse_position_msgs_.header.stamp = node_->get_clock()->now();
  mouse_position_msgs_.point.set__x(event.x);
  mouse_position_msgs_.point.set__y(event.y);
  mouse_position_pub_->publish(mouse_position_msgs_);
  return rviz_common::Tool::processMouseEvent(event);
}

}  // namespace hdmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hdmap_rviz_plugins::MouseTool, rviz_common::Tool)
