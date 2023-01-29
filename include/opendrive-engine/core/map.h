#ifndef OPENDRIVE_ENGINE_CORE_MAP_H_
#define OPENDRIVE_ENGINE_CORE_MAP_H_

#include "header.h"
#include "id.h"
#include "junction.h"
#include "road.h"

namespace opendrive {
namespace engine {
namespace core {

struct Map {
  Header::Ptr header;
  std::unordered_map<Id, Road::Ptr> roads;
  std::unordered_map<Id, Junction::Ptr> junctions;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_MAP_H_
