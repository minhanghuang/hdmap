#ifndef OPENDRIVE_ENGINE_ALGO_ROUTING_GRAPH_H_
#define OPENDRIVE_ENGINE_ALGO_ROUTING_GRAPH_H_

#include <limits>
#include <string>
#include <vector>

#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/id.h"
#include "opendrive-engine/core/lane.h"

namespace opendrive {
namespace engine {
namespace routing {

struct Node {
  /// node id
  core::Id id;
};

struct Path {
  Path()
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

#endif  // OPENDRIVE_ENGINE_ALGO_ROUTING_GRAPH_H_
