#ifndef HDMAP_RVIZ_PLUGIN_H_
#define HDMAP_RVIZ_PLUGIN_H_

#include <QGridLayout>
#include <QPushButton>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>  // context_->getRosNodeAbstraction().lock()->get_raw_node();
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_system.hpp>
#include <unordered_map>
#include <utility>

#include "hdmap_msgs/srv/get_global_map.hpp"

namespace hdmap_rviz_plugins {

class MapDisplay : public rviz_common::Display {
  Q_OBJECT
 public:
  MapDisplay();
  virtual ~MapDisplay();

 protected:
  virtual void onInitialize() override;

 private:
  void SetupRosService();

  void SetupRosSubscriptions();

  void MousePositionCallback(
      const geometry_msgs::msg::PointStamped::SharedPtr msg);

  void ShowGlobalMap();

  void GlobalMapMsgToBillboardLines(
      const hdmap_msgs::msg::Map& map,
      std::vector<std::shared_ptr<rviz_rendering::BillboardLine>>& lines);

  std::mutex mutex_;

  /**
   * @brief ros node
   */
  rclcpp::Node::SharedPtr node_;

  const std::string global_map_topic_;
  rclcpp::Client<hdmap_msgs::srv::GetGlobalMap>::SharedPtr global_map_client_;

  /// Display show lanes
  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> rviz_lines_;

  /// mouse position
  std::mutex mouse_position_mutex_;
  const std::string mouse_position_topic_;
  geometry_msgs::msg::PointStamped mouse_position_msgs_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      mouse_position_sub_;
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_H_
