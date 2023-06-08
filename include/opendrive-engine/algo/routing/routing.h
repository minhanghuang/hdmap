#ifndef OPENDRIVE_ENGINE_ALGO_ROUTING_H_
#define OPENDRIVE_ENGINE_ALGO_ROUTING_H_

#include <memory>
#include <string>
#include <vector>

#include "opendrive-engine/algo/kdtree/kdtree.h"
#include "opendrive-engine/algo/routing/astar_strategy.h"
#include "opendrive-engine/algo/routing/define.h"
#include "opendrive-engine/algo/routing/strategy.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/geometry/geometry.h"

namespace opendrive {
namespace engine {
namespace routing {

class Routing {
 public:
  Routing();
  Result Start(const Point& src_point, const Point& dest_point);

 private:
  std::vector<Node> SearchNearestPoints(const Point& point);

  /// topo
  core::Map::ConstPtr map_;

  /// knn
  kdtree::KDTree::Ptr kdtree_;

  /// planning
  std::unique_ptr<Strategy> strategy_;
};

}  // namespace routing
}  // namespace engine
}  // namespace opendrive

#endif  //  OPENDRIVE_ENGINE_ALGO_ROUTING_H_
