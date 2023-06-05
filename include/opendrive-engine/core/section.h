#ifndef OPENDRIVE_ENGINE_CORE_SECTION_H_
#define OPENDRIVE_ENGINE_CORE_SECTION_H_

#include <cactus/macros.h>

#include <memory>
#include <vector>

#include "id.h"
#include "lane.h"

namespace opendrive {
namespace engine {
namespace core {

class Section {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, id);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, parent_id);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, start_position, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, end_position, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, length, 0);
  CACTUS_REGISTER_MEMBER_SHARED_PTR(Lane, center_lane);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Lane::Ptrs, left_lanes);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Lane::Ptrs, right_lanes);

 public:
  using Ptr = std::shared_ptr<Section>;
  using ConstPtr = std::shared_ptr<Section const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Section() {}
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_SECTION_H_
