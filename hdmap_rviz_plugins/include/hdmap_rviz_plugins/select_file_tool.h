#ifndef HDMAP_RVIZ_PLUGIN_SELECT_FILE_TOOL_H_
#define HDMAP_RVIZ_PLUGIN_SELECT_FILE_TOOL_H_

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreViewport.h>

#include <QFileDialog>
#include <QGridLayout>
#include <QPushButton>
#include <QVBoxLayout>
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
#include <rviz_common/view_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_system.hpp>
#include <shared_mutex>
#include <vector>

#include "hdmap_common/fs.h"
#include "hdmap_common/rw_lock.h"
#include "hdmap_common/util.h"
#include "hdmap_msgs/msg/map_file_info.hpp"
#include "hdmap_msgs/srv/get_global_map.hpp"
#include "util/event_manager.h"

namespace hdmap_rviz_plugins {

class SelectFileTool : public rviz_common::Tool {
  Q_OBJECT
 public:
  /**
   * @brief Constructor
   */
  SelectFileTool();

  /**
   * @brief Destructor
   */
  virtual ~SelectFileTool();

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

 private:
  bool ProcessButton();

  rclcpp::Node::SharedPtr node_;

  std::shared_mutex mutex_;

  /// mouse position
  geometry_msgs::msg::PointStamped mouse_position_msgs_;

  /// select file
  hdmap_msgs::msg::MapFileInfo map_file_info_msg_;
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_SELECT_FILE_TOOL_H_