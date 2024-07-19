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
#include <vector>

#include "overlay_text.h"
#include "overlay_ui.h"
#include "overlay_utils.h"

namespace hdmap_rviz_plugins {

using namespace rviz_2d_overlay_plugins;

class OverlayComponent {
 public:
  using Ptr = std::shared_ptr<OverlayComponent>;
  using ConstPtr = std::shared_ptr<OverlayComponent const>;
  OverlayComponent(const std::string& name);

  void Clean();

  void Append(const std::string& data);

  void Update(const std::vector<std::string>& data);

  void Update(OverlayUI* ui);

  void Show();

  const std::string& name() const;

  void SetPosition(
      double hor_dist, double ver_dist,
      HorizontalAlignment hor_alignment = HorizontalAlignment::LEFT,
      VerticalAlignment ver_alignment = VerticalAlignment::TOP);

 private:
  std::mutex mutex_;

  const std::string name_;

  std::shared_ptr<OverlayText> text_;

  std::shared_ptr<OverlayObject> object_;
};

}  // namespace hdmap_rviz_plugins

#endif  // RVIZ_PLUGIN_OVERLAY_COMPONENT_H_
