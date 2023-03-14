#ifndef OPENDRIVE_ENGINE_PARAM_H_
#define OPENDRIVE_ENGINE_PARAM_H_

#include <memory>
#include <string>

namespace opendrive {
namespace engine {
namespace common {

struct Param {
  typedef std::shared_ptr<Param> Ptr;
  typedef std::shared_ptr<Param const> ConstPtr;
  Param() : map_file(""), step(0.5) {}
  std::string map_file;
  float step;
};

}  // namespace common
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_PARAM_H_
