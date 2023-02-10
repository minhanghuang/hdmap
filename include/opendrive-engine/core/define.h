#ifndef OPENDRIVE_ENGINE_DEFINE_H_
#define OPENDRIVE_ENGINE_DEFINE_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "map.h"

namespace opendrive {
namespace engine {
namespace core {

typedef std::unordered_map<Id, Lane::Ptr> LaneRoute;
typedef std::unordered_map<Id, Section::Ptr> SectionRoute;
typedef std::unordered_map<Id, Road::Ptr> RoadRoute;
typedef std::unordered_map<Id, Junction::Ptr> JunctionRoute;

typedef struct Data DataTypedef;
struct Data {
  typedef std::shared_ptr<DataTypedef> Ptr;
  typedef std::shared_ptr<DataTypedef const> ConstPtr;
  Header::Ptr header;
  LaneRoute lanes;
  SectionRoute sections;
  RoadRoute roads;
  JunctionRoute junctions;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_DEFINE_H_
