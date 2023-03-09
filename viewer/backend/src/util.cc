#include "util.h"

#include <algorithm>

namespace opendrive {
namespace engine {
namespace server {

bool ConvertLaneToPts(core::Lane::ConstPtr lane, Json& lane_json) {
  int lane_length =
      std::min(std::min(lane->central_curve().pts().size(),
                        lane->left_boundary().curve().pts().size()),
               lane->right_boundary().curve().pts().size());
  for (int i = 0; i < lane_length; i++) {
    lane_json[0][i][0] = lane->left_boundary().curve().pts().at(i).x();
    lane_json[0][i][1] = lane->left_boundary().curve().pts().at(i).y();
    lane_json[1][i][0] = lane->right_boundary().curve().pts().at(i).x();
    lane_json[1][i][1] = lane->right_boundary().curve().pts().at(i).y();
  }
  return true;
}

bool ConvertSectionToPts(core::Section::ConstPtr section, Json& section_json) {
  for (const auto& lane : section->left_lanes()) {
    Json lane_json;
    ConvertLaneToPts(lane, lane_json);
    section_json.emplace_back(lane_json);
  }
  Json lane_json;
  ConvertLaneToPts(section->center_lane(), lane_json);
  section_json.emplace_back(lane_json);
  for (const auto& lane : section->right_lanes()) {
    Json lane_json;
    ConvertLaneToPts(lane, lane_json);
    section_json.emplace_back(lane_json);
  }
  return true;
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
