#ifndef HDMAP_RVIZ_PLUGIN_OVERLAY_COMPONENT_H_
#define HDMAP_RVIZ_PLUGIN_OVERLAY_COMPONENT_H_

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

template <class UI>
class OverlayComponent {
 public:
  using Ptr = std::shared_ptr<OverlayComponent>;
  using ConstPtr = std::shared_ptr<OverlayComponent const>;

  OverlayComponent(const std::string& name)
      : name_(name),
        text_(std::make_shared<OverlayText>()),
        object_(std::make_shared<OverlayObject>(name_)),
        ui_(std::make_shared<UI>()) {}

  void Clean() {
    std::lock_guard<std::mutex> guard(mutex_);
    text_->mutable_data().clear();
  }

  void Append(const std::string& data) {
    std::lock_guard<std::mutex> guard(mutex_);
    text_->mutable_data().emplace_back(data);
  }

  void Update(const std::vector<std::string>& data) {
    std::lock_guard<std::mutex> guard(mutex_);
    text_->set_data(data);
  }

  void Show() {
    unsigned int hor_size = 1;
    unsigned int ver_size = 1;
    text_->mutable_data().clear();
    text_->set_data(ui_->Format());
    for (const auto& data : text_->data()) {
      if (!data.empty() && hor_size < data.size()) {
        hor_size = data.size();
      }
    }
    ver_size = std::max<unsigned int>(1, text_->data().size());
    object_->updateTextureSize(text_->CalcWidth(hor_size),
                               text_->CalcHeight(ver_size));

    rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = object_->getBuffer();
    QImage image = buffer.getQImage(*object_, text_->mutable_bg_color());
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(text_->fg_color(), std::max(text_->line_width(), 1),
                        Qt::SolidLine));
    const uint16_t text_width = object_->getTextureWidth();

    // font
    if (text_->font_size() > 0) {
      QFont font(text_->font().length() > 0 ? text_->font().c_str()
                                            : "Liberation Sans");
      font.setPointSize(text_->font_size());
      font.setBold(true);
      painter.setFont(font);
    }

    if (!text_->data().empty()) {
      QColor shadow_color;
      shadow_color = Qt::black;
      shadow_color.setAlpha(text_->fg_color().alpha());

      std::string color_wrapped_text;
      for (const auto& row : text_->data()) {
        color_wrapped_text +=
            (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, "
                           "%5%)\">%1%</span> <br>") %
             row % text_->fg_color().red() % text_->fg_color().green() %
             text_->fg_color().blue() % text_->fg_color().alpha())
                .str();
      }

      QStaticText static_text(
          boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >")
              .c_str());
      static_text.setTextWidth(text_width);

      painter.setPen(
          QPen(shadow_color, std::max(text_->line_width(), 1), Qt::SolidLine));
      painter.drawStaticText(0, 0, static_text);
    }
    painter.end();

    object_->setDimensions(object_->getTextureWidth(),
                           object_->getTextureHeight());
    object_->show();
  }

  void SetPosition(
      double hor_dist, double ver_dist,
      HorizontalAlignment hor_alignment = HorizontalAlignment::LEFT,
      VerticalAlignment ver_alignment = VerticalAlignment::TOP) {
    object_->setPosition(hor_dist, ver_dist, hor_alignment, ver_alignment);
  }

  const std::string& name() const { return name_; }

  std::shared_ptr<UI> ui() const { return ui_; }

 private:
  std::mutex mutex_;

  const std::string name_;

  std::shared_ptr<OverlayText> text_;

  std::shared_ptr<OverlayObject> object_;

  std::shared_ptr<UI> ui_;
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_OVERLAY_COMPONENT_H_