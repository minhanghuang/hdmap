#ifndef OPENDRIVE_ENGINE_CORE_ROAD_H_
#define OPENDRIVE_ENGINE_CORE_ROAD_H_

#include <opendrive-cpp/geometry/enums.h>

#include <vector>

#include "id.h"
#include "section.h"

namespace opendrive {
namespace engine {
namespace core {

struct RoadInfo {
  double s = 0.;
  RoadType type = RoadType::TOWN;
  double speed_limit;  // meters per second
};
typedef std::vector<RoadInfo> RoadInfos;

typedef struct Road RoadTypedef;
struct Road {
  typedef std::shared_ptr<RoadTypedef> Ptr;
  typedef std::shared_ptr<RoadTypedef const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  Id id;
  std::string name;
  Id junction_id;
  double length = 0.;
  Section::Ptrs sections;
  Ids predecessor_id;
  Ids successor_id;
  RoadRule rule = RoadRule::RHT;
  RoadInfos info;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_ROAD_H_
