#include "opendrive-engine/algo/routing/routing.h"

namespace opendrive {
namespace engine {
namespace routing {

Routing::Routing() {
  auto factory = cactus::Factory::Instance();
  kdtree_ = factory->GetObject<kdtree::KDTree>(kGlobalKdtreeObjectKey);
  strategy_ = std::make_unique<AStarStrategy>();
}

Result Routing::Start(const Point& src_point, const Point& dest_point) {
  return Result();
}

std::vector<Node> Routing::SearchNearestPoints(const Point& point) {
  std::vector<Node> nodes;

  kdtree::SearchResults nearest_points =
      kdtree_->Query(point.x(), point.y(), 50);

  // radius

  // heading

  return nodes;
}

}  // namespace routing
}  // namespace engine
}  // namespace opendrive
