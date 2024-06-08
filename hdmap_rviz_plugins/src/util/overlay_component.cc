#include "util/overlay_component.h"

namespace hdmap_rviz_plugins {

OverlayComponent::OverlayComponent()
    : text_(std::make_shared<OverlayText>()),
      object_(std::make_shared<OverlayObject>("OverlayTextDisplay")) {}

void OverlayComponent::Clean() {
  std::lock_guard<std::mutex> guard(mutex_);
  text_->mutable_data().clear();
}

void OverlayComponent::Append(const std::string& data) {
  std::lock_guard<std::mutex> guard(mutex_);
  text_->mutable_data().emplace_back(data);
}

void OverlayComponent::Update(const std::vector<std::string>& data) {
  std::lock_guard<std::mutex> guard(mutex_);
  text_->set_data(data);
}

void OverlayComponent::Update(OverlayUI* ui) {
  if (nullptr == ui) {
    return;
  }
  std::lock_guard<std::mutex> guard(mutex_);
  text_->mutable_data().clear();
  text_->set_data(ui->Format());
}

void OverlayComponent::Show() {
  object_->updateTextureSize(
      text_->CalcWidth(text_->data().at(1).size()),
      text_->CalcHeight(text_->data().size()));
  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = object_->getBuffer();
  QImage image = buffer.getQImage(*object_, text_->mutable_bg_color());
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(
      QPen(text_->fg_color(), std::max(text_->line_width(), 1), Qt::SolidLine));
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

}  // namespace hdmap_rviz_plugins
