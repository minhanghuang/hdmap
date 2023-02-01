#ifndef OPENDRIVE_ENGINE_CORE_ID_H_
#define OPENDRIVE_ENGINE_CORE_ID_H_

#include <string>
#include <unordered_set>

#include "opendrive-cpp/opendrive.h"

namespace opendrive {
namespace engine {
namespace core {

typedef std::string Id;
typedef std::unordered_set<Id> Ids;
typedef std::vector<Id> Path;

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_ID_H_
