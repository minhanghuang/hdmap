#ifndef OPENDRIVE_ENGINE_ALGO_ROUTING_DEFINE_H_
#define OPENDRIVE_ENGINE_ALGO_ROUTING_DEFINE_H_

#include <limits>
#include <string>
#include <vector>

#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/id.h"
#include "opendrive-engine/core/lane.h"
#include "opendrive-engine/geometry/geometry.h"

namespace opendrive {
namespace engine {
namespace routing {

using Point = geometry::Point4D;

struct Node {
  /// point
  core::Curve::Point point;
};

struct Result {
  Result()
      : status(ErrorCode::kRoutingError),
        cost(std::numeric_limits<double>::max()) {}

  /// planning status
  Status status;

  /// planning path
  std::vector<core::Id> path;

  /// planning cost
  double cost;
};

}  // namespace routing
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_ALGO_ROUTING_DEFINE_H_
