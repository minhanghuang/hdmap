#include "hdmap/algo/routing/astar_strategy.h"

namespace hdmap {
namespace routing {

AStarStrategy::AStarStrategy() : Strategy() {}

Strategy::Result AStarStrategy::Search(const Strategy::Node& src_node,
                                       const Strategy::Node& dest_node) {
  Result result;
  return result;
}

}  // namespace routing
}  // namespace hdmap
