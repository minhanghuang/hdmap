#ifndef HDMAP_ENGINE_CORE_ROUTING_H_
#define HDMAP_ENGINE_CORE_ROUTING_H_

#include <memory>
#include <string>
#include <vector>

#include "hdmap_engine/common/status.h"
#include "hdmap_engine/core/kdtree/kdtree.h"
#include "hdmap_engine/core/routing/astar_strategy.h"
#include "hdmap_engine/core/routing/strategy.h"
#include "hdmap_engine/geometry.h"

namespace hdmap {
namespace routing {

class Routing {
 public:
  Routing();
  Strategy::Result Start(const Strategy::Point& src_point,
                         const Strategy::Point& dest_point);

 private:
  std::vector<Strategy::Node> SearchNearestPoints(const Strategy::Point& point);

  /// topo
  geometry::Map::ConstPtr map_;

  /// knn
  kdtree::KDTree::Ptr kdtree_;

  /// planning
  std::unique_ptr<Strategy> strategy_;
};

}  // namespace routing
}  // namespace hdmap

#endif  //  HDMAP_ENGINE_CORE_ROUTING_H_
