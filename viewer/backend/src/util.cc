#include "util.h"

namespace opendrive {
namespace engine {
namespace server {

bool ConvertLaneToLaneMsg(const core::Lane::ConstPtr& lane,
                          msgs::Lane& lane_msg) {
  if (common::IsLineGeometry(lane)) {
    lane_msg.mutable_left_boundary()->mutable_pts()->emplace_back(
        msgs::Point(lane->left_boundary().curve().pts().front().x(),
                    lane->left_boundary().curve().pts().front().y()));
    lane_msg.mutable_left_boundary()->mutable_pts()->emplace_back(
        msgs::Point(lane->left_boundary().curve().pts().back().x(),
                    lane->left_boundary().curve().pts().back().y()));
    lane_msg.mutable_right_boundary()->mutable_pts()->emplace_back(
        msgs::Point(lane->right_boundary().curve().pts().front().x(),
                    lane->right_boundary().curve().pts().front().y()));
    lane_msg.mutable_right_boundary()->mutable_pts()->emplace_back(
        msgs::Point(lane->right_boundary().curve().pts().back().x(),
                    lane->right_boundary().curve().pts().back().y()));
  } else {
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
  }
  return true;
}

bool ConvertLaneToLanesMsg(const core::Lane::ConstPtrs& lanes,
                           msgs::Lanes& lanes_msg) {
  for (const auto& lane : lanes) {
    msgs::Lane lane_msg;
    ConvertLaneToLaneMsg(lane, lane_msg);
    lanes_msg.mutable_lanes()->emplace_back(lane_msg);
  }
  return true;
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
