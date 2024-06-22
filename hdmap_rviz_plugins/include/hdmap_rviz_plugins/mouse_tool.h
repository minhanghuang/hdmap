#ifndef MOUSE_RVIZ_PLUGIN_H_
#define MOUSE_RVIZ_PLUGIN_H_

#include <QGridLayout>
#include <QPushButton>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>  // context_->getRosNodeAbstraction().lock()->get_raw_node();
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_system.hpp>
#include <vector>

#include "hdmap_msgs/srv/get_global_map.hpp"

namespace hdmap_rviz_plugins {

class MouseTool : public rviz_common::Tool {
  Q_OBJECT
 public:
  /**
   * @brief Constructor
   */
  MouseTool();

  /**
   * @brief Destructor
   */
  virtual ~MouseTool();

  /**
   * @brief RViz callback on initialize
   */
  virtual void onInitialize() override;

  /**
   * @brief RViz callback for activating
   */
  virtual void activate() override;

  /**
   * @bríef RViz callback for deactivating
   */
  virtual void deactivate() override;

  /**
   * @brief RViz callback for mouse events
   * @param event The mouse event
   * @return Exit code
   */
  virtual int processMouseEvent(
      rviz_common::ViewportMouseEvent& event) override;

 private Q_SLOTS:

 private:
  rclcpp::Node::SharedPtr node_;

  std::mutex mutex_;

  const std::string mouse_position_topic_;
  geometry_msgs::msg::PointStamped mouse_position_msgs_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      mouse_position_pub_;
};

}  // namespace hdmap_rviz_plugins

#endif  // MOUSE_RVIZ_PLUGIN_H_
