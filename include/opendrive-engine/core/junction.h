#ifndef OPENDRIVE_ENGINE_CORE_JUNCTION_H_
#define OPENDRIVE_ENGINE_CORE_JUNCTION_H_

#include <cactus/macros.h>
#include <opendrive-cpp/opendrive.h>

#include <memory>

#include "opendrive-engine/core/id.h"
#include "opendrive-engine/geometry/geometry.h"

namespace opendrive {
namespace engine {
namespace core {

class Junction {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, id);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, name);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(JunctionType, type);

 public:
  using Ptr = std::shared_ptr<Junction>;
  using ConstPtr = std::shared_ptr<Junction const>;
  Junction() : type_(JunctionType::kDefault) {}
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_JUNCTION_H_
