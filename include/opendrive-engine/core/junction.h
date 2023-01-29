#ifndef OPENDRIVE_ENGINE_CORE_JUNCTION_H_
#define OPENDRIVE_ENGINE_CORE_JUNCTION_H_

#include <memory>

#include "geometry.h"
#include "id.h"

namespace opendrive {
namespace engine {
namespace core {

typedef struct Junction JunctionTypedef;
struct Junction {
  typedef std::shared_ptr<JunctionTypedef> Ptr;
  typedef std::shared_ptr<JunctionTypedef const> ConstPtr;
  Id id;
  std::string name;
  JunctionType type = JunctionType::DEFAULT;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_JUNCTION_H_
