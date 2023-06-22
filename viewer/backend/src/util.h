#ifndef OPENDRIVE_ENGINE_SERVER_UTIL_H_
#define OPENDRIVE_ENGINE_SERVER_UTIL_H_

#include <opendrive-engine/common/common.h>
#include <opendrive-engine/core/lane.h>
#include <opendrive-engine/core/section.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>

#include "msgs.h"

namespace opendrive {
namespace engine {
namespace server {

bool ConvertLaneToLaneMsg(const core::Lane::ConstPtr& lane,
                          msgs::Lane& lane_msg);

bool ConvertLaneToLanesMsg(const core::Lane::ConstPtrs& lanes,
                           msgs::Lanes& lanes_msg);

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_UTIL_H_
