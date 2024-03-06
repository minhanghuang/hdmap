#ifndef OPENDRIVE_ENGINE_SERVER_UTIL_H_
#define OPENDRIVE_ENGINE_SERVER_UTIL_H_

#include <hdmap/common/utils.h>
#include <hdmap/geometry.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>

#include "msgs.h"

namespace hdmap {
namespace server {

bool ConvertLaneToLaneMsg(const geometry::Lane::ConstPtr& lane,
                          msgs::Lane& lane_msg);

bool ConvertLaneToLanesMsg(const geometry::Lane::ConstPtrs& lanes,
                           msgs::Lanes& lanes_msg);

}  // namespace server
}  // namespace hdmap

#endif  // OPENDRIVE_ENGINE_SERVER_UTIL_H_
