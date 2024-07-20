#ifndef HDMAP_RVIZ_PLUGIN_HDMAP_DISPLAY_H_
#define HDMAP_RVIZ_PLUGIN_HDMAP_DISPLAY_H_

#include <QFontDatabase>
#include <QGridLayout>
#include <QPainter>
#include <QPushButton>
#include <QStaticText>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>  // context_->getRosNodeAbstraction().lock()->get_raw_node();
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/selection_handler.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/interactive_object.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_system.hpp>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hdmap_common/log.h"
#include "hdmap_common/rw_lock.h"
#include "hdmap_msgs/msg/map_file_info.hpp"
#include "hdmap_msgs/msg/region.hpp"
#include "hdmap_msgs/srv/get_global_map.hpp"
#include "hdmap_msgs/srv/send_map_file.hpp"
#include "util/current_region.h"
#include "util/event_manager.h"
#include "util/overlay_component.h"
#include "util/overlay_text.h"
#include "util/overlay_ui.h"

namespace hdmap_rviz_plugins {

class MapDisplay : public rviz_common::Display {
  Q_OBJECT
 public:
  using RvizLine = rviz_rendering::BillboardLine;
  using RvizLinePtr = std::shared_ptr<RvizLine>;
  using RvizLinaPtrs = std::vector<RvizLinePtr>;

  MapDisplay();
  virtual ~MapDisplay();

 protected:
  virtual void onInitialize() override;

 private:
  void SetupRosSubscriptions();

  void SetupRosPublisher();

  void SetupRosService();

  void SetupRosTimer();

  void SetupOverlay();

  void SetupRvizEvent();

  void CallGlobalMap();

  void CallSendMap(const hdmap_msgs::msg::MapFileInfo& map_file_info);

  void ShowCurrentRegion();

  void CurrentRegionCallback(const hdmap_msgs::msg::Region::SharedPtr msg);

  void HandleEventFromMouseCursor(void* msg);

  void HandleEventFromSelectFile(void* msg);

  void ConvertToBillboardLines(const hdmap_msgs::msg::Map& msg);

  std::mutex mutex_;

  /**
   * @brief ros node
   */
  rclcpp::Node::SharedPtr nh_;

  /// ros timer
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  /// ros service
  // global map
  const std::string global_map_topic_;
  rclcpp::Client<hdmap_msgs::srv::GetGlobalMap>::SharedPtr global_map_client_;
  // send map
  const std::string send_map_topic_;
  rclcpp::Client<hdmap_msgs::srv::SendMapFile>::SharedPtr send_map_client_;

  /// ros sub
  // current region
  const std::string current_region_topic_;
  std::shared_mutex current_region_mutex_;
  hdmap_msgs::msg::Region current_region_msg_;
  rclcpp::Subscription<hdmap_msgs::msg::Region>::SharedPtr current_region_sub_;

  /// ros pub
  // mouse position
  const std::string mouse_position_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      mouse_position_pub_;

  /// select file loaded
  std::atomic<bool> file_loaded_;
  std::shared_mutex map_file_info_mutex_;
  hdmap_msgs::msg::MapFileInfo map_file_info_;

  /// Display show lanes
  RvizLinaPtrs rviz_lines_;

  /// Display current lane
  std::shared_ptr<CurrentRegion> current_region_;

  /// Display overlay
  std::shared_ptr<OverlayComponent<CurrentRegionOverlayUI>>
      current_region_overlay_;
  std::shared_ptr<OverlayComponent<MousePositionOverlayUI>>
      mouse_position_overlay_;
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_HDMAP_DISPLAY_H_
