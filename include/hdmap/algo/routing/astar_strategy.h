#ifndef HDMAP_ALGO_ROUTING_ASTAR_STRATEGY_H_
#define HDMAP_ALGO_ROUTING_ASTAR_STRATEGY_H_

#include <string>
#include <vector>

#include "hdmap/algo/routing/strategy.h"
#include "hdmap/common/status.h"

namespace hdmap {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  AStarStrategy();

  virtual Result Search(const Node& src_node, const Node& dest_node) override;
};

}  // namespace routing
}  // namespace hdmap

#endif  // HDMAP_ALGO_ROUTING_ASTAR_STRATEGY_H_
