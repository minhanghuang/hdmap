#ifndef OPENDRIVE_ENGINE_CORE_MAP_H_
#define OPENDRIVE_ENGINE_CORE_MAP_H_

#include <cactus/macros.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "header.h"
#include "id.h"
#include "junction.h"
#include "lane.h"
#include "road.h"
#include "section.h"

namespace opendrive {
namespace engine {
namespace core {

using LaneRoute = std::unordered_map<Id, Lane::Ptr>;
using ConstLaneRoute = std::unordered_map<Id, Lane::ConstPtr>;
using SectionRoute = std::unordered_map<Id, Section::Ptr>;
using ConstSectionRoute = std::unordered_map<Id, Section::ConstPtr>;
using RoadRoute = std::unordered_map<Id, Road::Ptr>;
using ConstRoadRoute = std::unordered_map<Id, Road::ConstPtr>;
using JunctionRoute = std::unordered_map<Id, Junction::Ptr>;
using ConstJunctionRoute = std::unordered_map<Id, Junction::ConstPtr>;

class Map {
  CACTUS_REGISTER_MEMBER_SHARED_PTR(Header, header);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(LaneRoute, lanes);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(SectionRoute, sections);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(RoadRoute, roads);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(JunctionRoute, junctions);

 public:
  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<Map const>;
  Map() {}
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_MAP_H_
