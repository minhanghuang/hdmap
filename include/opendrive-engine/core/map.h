#ifndef OPENDRIVE_ENGINE_CORE_MAP_H_
#define OPENDRIVE_ENGINE_CORE_MAP_H_

#include <memory>

#include "header.h"
#include "id.h"
#include "junction.h"
#include "road.h"

namespace opendrive {
namespace engine {
namespace core {

typedef struct Map MapTypedef;
struct Map {
  typedef std::shared_ptr<MapTypedef> Ptr;
  typedef std::shared_ptr<MapTypedef const> ConstPtr;
  Header::Ptr header;
  std::unordered_map<Id, Road::Ptr> roads;
  std::unordered_map<Id, Junction::Ptr> junctions;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_MAP_H_
