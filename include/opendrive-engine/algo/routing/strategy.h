#ifndef OPENDRIVE_ENGINE_ALGO_ROUTING_STRATEGY_H_
#define OPENDRIVE_ENGINE_ALGO_ROUTING_STRATEGY_H_

#include <cactus/factory.h>

#include <memory>
#include <string>
#include <vector>

#include "opendrive-engine/algo/routing/define.h"
#include "opendrive-engine/common/g.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/map.h"

namespace opendrive {
namespace engine {
namespace routing {

class Strategy {
 public:
  using Ptr = std::shared_ptr<Strategy>;
  using ConstPtr = std::shared_ptr<Strategy const>;
  Strategy();
  virtual ~Strategy() = default;
  virtual Result Search(const Node& src_node, const Node& dest_node) = 0;

 protected:
  core::Map::ConstPtr map_;
};

}  // namespace routing
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_ALGO_ROUTING_STRATEGY_H_
