#include "hdmap/algo/routing/routing.h"

namespace hdmap {
namespace routing {

Routing::Routing() {}

Strategy::Result Routing::Start(const Strategy::Point& src_point,
                                const Strategy::Point& dest_point) {
  return Strategy::Result();
}

std::vector<Strategy::Node> Routing::SearchNearestPoints(
    const Strategy::Point& point) {
  std::vector<Strategy::Node> nodes;

  // kdtree::SearchResults nearest_points =
  // kdtree_->Query(point.x(), point.y(), 50);

  // radius

  // heading

  return nodes;
}

}  // namespace routing
}  // namespace hdmap
