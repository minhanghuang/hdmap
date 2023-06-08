#ifndef OPENDRIVE_ENGINE_ALGO_ROUTING_ASTAR_STRATEGY_H_
#define OPENDRIVE_ENGINE_ALGO_ROUTING_ASTAR_STRATEGY_H_

#include <string>
#include <vector>

#include "opendrive-engine/algo/routing/define.h"
#include "opendrive-engine/algo/routing/strategy.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/map.h"

namespace opendrive {
namespace engine {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  AStarStrategy();
  virtual Result Search(const Node& src_node, const Node& dest_node) override;
};

}  // namespace routing
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_ALGO_ROUTING_ASTAR_STRATEGY_H_
