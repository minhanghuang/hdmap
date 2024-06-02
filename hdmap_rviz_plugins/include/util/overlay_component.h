#ifndef RVIZ_PLUGIN_OVERLAY_COMPONENT_H_
#define RVIZ_PLUGIN_OVERLAY_COMPONENT_H_

#include <QColor>
#include <QFontDatabase>
#include <QGridLayout>
#include <QPainter>
#include <QPushButton>
#include <QStaticText>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <hdmap_msgs/srv/get_global_map.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>

#include "util/overlay_text.h"
#include "util/overlay_utils.h"

namespace hdmap_rviz_plugins {

using namespace rviz_2d_overlay_plugins;

class OverlayComponent {
 public:
  OverlayComponent();

  void Clean();

  void Append(const std::string& data);

  void Update(const std::string& data);

  void Show();

 private:
  std::mutex mutex_;

  std::shared_ptr<OverlayText> text_;

  std::shared_ptr<OverlayObject> object_;
};

}  // namespace hdmap_rviz_plugins

#endif  // RVIZ_PLUGIN_OVERLAY_COMPONENT_H_
