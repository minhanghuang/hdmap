#ifndef HDMAP_ENGINE_CORE_ROUTING_STRATEGY_H_
#define HDMAP_ENGINE_CORE_ROUTING_STRATEGY_H_

#include <memory>
#include <string>
#include <vector>

#include "hdmap_engine/common/status.h"
#include "hdmap_engine/geometry.h"

namespace hdmap {
namespace routing {

class Strategy {
 public:
  using Ptr = std::shared_ptr<Strategy>;
  using ConstPtr = std::shared_ptr<Strategy const>;
  using Point = geometry::Point4D;

  struct Node {
    /// point
    geometry::Curve::Point point;
  };

  struct Result {
    Result() : cost(std::numeric_limits<double>::max()) {}

    /// planning path
    std::vector<std::string> path;

    /// planning cost
    double cost;
  };

  Strategy();

  virtual ~Strategy() = default;

  virtual Result Search(const Node& src_node, const Node& dest_node) = 0;

 protected:
  geometry::Map::ConstPtr map_;
};

}  // namespace routing
}  // namespace hdmap

#endif  // HDMAP_ENGINE_CORE_ROUTING_STRATEGY_H_
