#ifndef HDMAP_ENGINE_CORE_ROUTING_ASTAR_STRATEGY_H_
#define HDMAP_ENGINE_CORE_ROUTING_ASTAR_STRATEGY_H_

#include <string>
#include <vector>

#include "hdmap_engine/common/status.h"
#include "hdmap_engine/core/routing/strategy.h"

namespace hdmap {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  AStarStrategy();

  virtual Result Search(const Node& src_node, const Node& dest_node) override;
};

}  // namespace routing
}  // namespace hdmap

#endif  // HDMAP_ENGINE_CORE_ROUTING_ASTAR_STRATEGY_H_
