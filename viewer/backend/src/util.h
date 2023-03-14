#ifndef OPENDRIVE_ENGINE_SERVER_UTIL_H_
#define OPENDRIVE_ENGINE_SERVER_UTIL_H_

#include <opendrive-engine/common/common.h>
#include <opendrive-engine/core/lane.h>
#include <opendrive-engine/core/section.h>

#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>

namespace opendrive {
namespace engine {
namespace server {

typedef nlohmann::json Json;
typedef nlohmann::json::value_t JsonValueType;
typedef std::string Data;
typedef std::unordered_map<std::string, JsonValueType> RequiredKeys;

bool ConvertLineToPts(const core::Curve& line, Json& line_json);

bool ConvertLaneToPts(core::Lane::ConstPtr lane, Json& data);

bool ConvertLaneToSimplePts(core::Lane::ConstPtr lane, Json& data);

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_UTIL_H_
