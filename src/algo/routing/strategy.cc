#include "opendrive-engine/algo/routing/strategy.h"

namespace opendrive {
namespace engine {
namespace routing {

Strategy::Strategy() {
  map_ = cactus::Factory::Instance()->GetObject<core::Map>(
      kGlobalCoreMapObjectKey);
}

}  // namespace routing
}  // namespace engine
}  // namespace opendrive
