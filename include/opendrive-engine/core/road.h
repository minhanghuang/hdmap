#ifndef OPENDRIVE_ENGINE_CORE_ROAD_H_
#define OPENDRIVE_ENGINE_CORE_ROAD_H_

#include <cactus/macros.h>
#include <opendrive-cpp/geometry/enums.h>

#include <vector>

#include "id.h"
#include "section.h"

namespace opendrive {
namespace engine {
namespace core {

class RoadInfo {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, start_position, 0);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(RoadType, type);
  // meters per second
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, speed_limit, 0);

 public:
  RoadInfo() : type_(RoadType::kTown) {}
};
typedef std::vector<RoadInfo> RoadInfos;

class Road {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, id);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, name);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, junction_id);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, length, 0);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(RoadRule, rule);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Section::Ptrs, sections);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Ids, predecessor_ids);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Ids, successor_ids);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(RoadInfos, info);

 public:
  using Ptr = std::shared_ptr<Road>;
  using ConstPtr = std::shared_ptr<Road const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Road() : rule_(RoadRule::kRht) {}
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_ROAD_H_
