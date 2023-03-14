#include "util.h"

#include <algorithm>

namespace opendrive {
namespace engine {
namespace server {

bool ConvertLineToPts(const core::Curve& line, Json& line_json) {
  line_json.clear();
  for (int i = 0; i < line.pts().size(); i++) {
    line_json[i][0] = line.pts().at(i).x();
    line_json[i][1] = line.pts().at(i).y();
  }
  return true;
}

bool ConvertLaneToPts(core::Lane::ConstPtr lane, Json& data) {
  Json left_line;
  Json right_line;
  int pts_size =
      std::min(lane->central_curve().pts().size(),
               std::min(lane->left_boundary().curve().pts().size(),
                        lane->right_boundary().curve().pts().size()));
  for (int i = 0; i < pts_size; i++) {
    left_line[i][0] = lane->left_boundary().curve().pts().at(i).x();
    left_line[i][1] = lane->left_boundary().curve().pts().at(i).y();
    right_line[i][0] = lane->right_boundary().curve().pts().at(i).x();
    right_line[i][1] = lane->right_boundary().curve().pts().at(i).y();
  }
  data.emplace_back(left_line);
  data.emplace_back(right_line);
  return true;
}

bool ConvertLaneToSimplePts(core::Lane::ConstPtr lane, Json& data) {
  Json left_line;
  Json right_line;
  if (common::IsLineGeometry(lane)) {
    // 只取头尾两个点
    left_line[0][0] = lane->left_boundary().curve().pts().front().x();
    left_line[0][1] = lane->left_boundary().curve().pts().front().y();
    left_line[1][0] = lane->left_boundary().curve().pts().back().x();
    left_line[1][1] = lane->left_boundary().curve().pts().back().y();

    right_line[0][0] = lane->right_boundary().curve().pts().front().x();
    right_line[0][1] = lane->right_boundary().curve().pts().front().y();
    right_line[1][0] = lane->right_boundary().curve().pts().back().x();
    right_line[1][1] = lane->right_boundary().curve().pts().back().y();
    data.emplace_back(left_line);
    data.emplace_back(right_line);
  } else {
    ConvertLaneToPts(lane, data);
  }
  return true;
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
