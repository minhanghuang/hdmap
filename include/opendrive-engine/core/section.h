#ifndef OPENDRIVE_ENGINE_CORE_SECTION_H_
#define OPENDRIVE_ENGINE_CORE_SECTION_H_

#include <memory>
#include <vector>

#include "id.h"
#include "lane.h"

namespace opendrive {
namespace engine {
namespace core {

typedef struct Section SectionTypedef;
struct Section {
  typedef std::shared_ptr<SectionTypedef> Ptr;
  typedef std::shared_ptr<SectionTypedef const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  Id id;
  Id parent_id;  // road id
  double s0 = 0.;
  double s1 = 0.;
  double length = 0.;
  Lane::Ptr center_lane;
  Lane::Ptrs left_lanes;
  Lane::Ptrs right_lanes;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_SECTION_H_
