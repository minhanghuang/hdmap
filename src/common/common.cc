#include "opendrive-engine/common/common.h"

namespace opendrive {
namespace engine {
namespace common {

core::Id GetLaneIdById(const core::Id& point_id) {
  core::Id lane_id;
  auto split_ret = cactus::StrSplit(point_id, "_");
  if (5 != split_ret.size()) {
    return lane_id;
  }
  return split_ret[0] + "_" + split_ret[1] + "_" + split_ret[2];
}

bool IsLineGeometry(core::Lane::ConstPtr lane) {
  if (lane->geometrys().empty()) {
    return false;
  }
  for (const auto& geometry : lane->geometrys()) {
    if (core::Geometry::Type::kLine != geometry.type()) {
      return false;
    }
  }
  return true;
}

}  // namespace common
}  // namespace engine
}  // namespace opendrive
