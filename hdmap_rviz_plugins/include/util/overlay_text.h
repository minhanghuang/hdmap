#ifndef RVIZ_PLUGIN_OVERLAY_TEXT_H_
#define RVIZ_PLUGIN_OVERLAY_TEXT_H_

#include <QColor>
#include <string>

namespace hdmap_rviz_plugins {

class OverlayText {
 public:
  OverlayText()
      : height_(40),
        width_(200),
        bg_color_(QColor(0, 0, 0, 0)),  // black
        fg_color_(QColor(255, 100, 0)),
        font_("Liberation Sans"),
        font_size_(14),
        line_width_(12),
        data_("") {}

  int height() const { return height_; }

  void set_height(int height) { height_ = height; }

  int width() const { return width_; }

  void set_width(int width) { width_ = width; }

  QColor bg_color() const { return bg_color_; }

  QColor& mutable_bg_color() { return bg_color_; }

  void set_bg_color(const QColor& bg_color) { bg_color_ = bg_color; }

  QColor fg_color() const { return fg_color_; }

  QColor& mutable_fg_color() { return fg_color_; }

  void set_fg_color(const QColor& fg_color) { fg_color_ = fg_color; }

  std::string font() const { return font_; }

  void set_font(const std::string& font) { font_ = font; }

  int font_size() const { return font_size_; }

  void set_font_size(int font_size) { font_size_ = font_size; }

  int line_width() const { return line_width_; }

  void set_line_width(int line_width) { line_width_ = line_width; }

  std::string data() const { return data_; }

  void set_data(const std::string& data) { data_ = data; }

  std::string& mutable_data() { return data_; }

 private:
  /**
   * @brief texture height
   */
  int height_;

  /**
   * @brief texture width
   */
  int width_;

  /**
   * @brief background color
   */
  QColor bg_color_;

  /**
   * @brief font color
   */
  QColor fg_color_;

  /**
   * @brief font
   */
  std::string font_;

  /**
   * @brief text font size
   */
  int font_size_;

  /**
   * @brief text line width
   */
  int line_width_;

  /**
   * @brief display text
   */
  std::string data_;
};

}  // namespace hdmap_rviz_plugins

#endif  // RVIZ_PLUGIN_OVERLAY_TEXT_H_
