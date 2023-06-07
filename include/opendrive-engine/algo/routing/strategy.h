#ifndef OPENDRIVE_ENGINE_ALGO_ROUTING_STRATEGY_H_
#define OPENDRIVE_ENGINE_ALGO_ROUTING_STRATEGY_H_

#include <string>
#include <vector>

#include "opendrive-engine/common/status.h"

namespace opendrive {
namespace engine {
namespace routing {

class Strategy {
 public:
  Strategy();
  virtual ~Strategy() = default;
  virtual Status Search() = 0;

 private:
};

}  // namespace routing
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_ALGO_ROUTING_STRATEGY_H_
