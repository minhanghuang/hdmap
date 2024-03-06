#include "util.h"

namespace hdmap {
namespace server {

bool ConvertLaneToLaneMsg(const geometry::Lane::ConstPtr& lane,
                          msgs::Lane& lane_msg) {
  int pts_size = std::min(lane->left_boundary().curve().pts().size(),
                          lane->right_boundary().curve().pts().size());
  for (int i = 0; i < pts_size; i++) {
    lane_msg.mutable_left_boundary()->mutable_pts()->emplace_back(
        msgs::Point(lane->left_boundary().curve().pts()[i].x(),
                    lane->left_boundary().curve().pts()[i].y()));
    lane_msg.mutable_right_boundary()->mutable_pts()->emplace_back(
        msgs::Point(lane->right_boundary().curve().pts()[i].x(),
                    lane->right_boundary().curve().pts()[i].y()));
  }

  return true;
}

bool ConvertLaneToLanesMsg(const geometry::Lane::ConstPtrs& lanes,
                           msgs::Lanes& lanes_msg) {
  for (const auto& lane : lanes) {
    msgs::Lane lane_msg;
    ConvertLaneToLaneMsg(lane, lane_msg);
    lanes_msg.mutable_lanes()->emplace_back(lane_msg);
  }
  return true;
}

}  // namespace server
}  // namespace hdmap
