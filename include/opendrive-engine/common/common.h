#ifndef OPENDRIVE_ENGINE_COMMON_H_
#define OPENDRIVE_ENGINE_COMMON_H_

#include <iostream>

#include "cactus/cactus.h"
#include "opendrive-engine/core/id.h"
#include "opendrive-engine/core/lane.h"

namespace opendrive {
namespace engine {
namespace common {

core::Id GetLaneIdById(const core::Id& point_id);

bool IsLineGeometry(core::Lane::ConstPtr lane);

}  // namespace common
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_COMMON_H_
