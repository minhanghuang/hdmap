#ifndef OPENDRIVE_ENGINE_PARAM_H_
#define OPENDRIVE_ENGINE_PARAM_H_

#include <memory>
#include <string>

namespace opendrive {
namespace engine {
namespace common {

struct Param {
  using Ptr = std::shared_ptr<Param>;
  using ConstPtr = std::shared_ptr<Param const>;
  Param() : map_file(""), step(0.5) {}
  std::string map_file;
  float step;
};

}  // namespace common
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_PARAM_H_
