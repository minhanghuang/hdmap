#ifndef HDMAP_RVIZ_PLUGIN_OVERLAY_UI_H_
#define HDMAP_RVIZ_PLUGIN_OVERLAY_UI_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "hdmap_common/util.h"

namespace hdmap_rviz_plugins {

/**
 * @class OverlayUI
 * @brief
 *    key: double
 *    key: std::string
 *    key: std::vector<double>
 *    key: std::vector<std::string>
 *
 */
class OverlayUI {
 public:
  OverlayUI() = default;
  virtual ~OverlayUI() = default;
  virtual std::vector<std::string> Format() = 0;
};

class CurrentRegionOverlayUI : public OverlayUI {
 public:
  CurrentRegionOverlayUI() { point_.reserve(3); }
  virtual std::vector<std::string> Format() override {
    std::vector<std::string> ret;
    // file path
    std::string file_path_text;
    file_path_text.append("file: ");
    file_path_text.append(hdmap::common::ShortenPath(file_path_));
    ret.emplace_back(file_path_text);

    // id
    std::string id_text;
    id_text.append("id: ");
    id_text.append(id_);
    ret.emplace_back(id_text);

    // point
    std::string point_text;
    point_text.append("point: [");
    for (int i = 0; i < 2; i++) {
      point_text.append(std::to_string(point_.at(i)));
      point_text.append("  ");
    }
    point_text.append("]");
    ret.emplace_back(point_text);

    // s
    std::string s_text;
    s_text.append("s: ");
    s_text.append(std::to_string(s_));
    ret.emplace_back(s_text);

    // heading
    std::string heading_text;
    heading_text.append("heading: ");
    heading_text.append(std::to_string(point_.at(2)));
    ret.emplace_back(heading_text);
    return ret;
  }

  std::string file_path() const { return file_path_; }

  std::string* mutable_file_path() { return &file_path_; }

  std::string id() const { return id_; }

  std::string* mutable_id() { return &id_; }

  std::vector<double> point() const { return point_; }

  std::vector<double>* mutable_point() { return &point_; }

  double s() const { return s_; }

  void set_s(double s) { s_ = s; }

 private:
  std::string file_path_;  // map file path(abs)

  std::string id_;  // lane id

  std::vector<double> point_;  // [x, y, heading]

  double s_;
};

class MousePositionOverlayUI : public OverlayUI {
 public:
  MousePositionOverlayUI() : x_(0), y_(0), z_(0) {}

  virtual std::vector<std::string> Format() override {
    std::vector<std::string> text;  // 1.1, 2.3, 0.1

    std::string xyz;
    xyz.append(std::to_string(x_));
    xyz.append(", ");
    xyz.append(std::to_string(y_));
    xyz.append(", ");
    xyz.append(std::to_string(z_));
    text.emplace_back(xyz);

    return text;
  }

  void set_x(double x) { x_ = x; }

  void set_y(double y) { y_ = y; }

  void set_z(double z) { z_ = z; }

 private:
  double x_;

  double y_;

  double z_;
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_OVERLAY_UI_H_
