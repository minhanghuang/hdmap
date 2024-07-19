#include "hdmap_rviz_plugins/util/current_region.h"

namespace hdmap_rviz_plugins {

std::string CurrentRegion::lane_id() { return lane_id_; }

void CurrentRegion::set_lane_id(const std::string& lane_id) {
  lane_id_ = lane_id;
}

std::shared_ptr<rviz_rendering::BillboardLine>
CurrentRegion::mutable_boundary() {
  return boundary_;
}

}  // namespace hdmap_rviz_plugins
