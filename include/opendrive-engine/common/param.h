#ifndef OPENDRIVE_ENGINE_PARAM_H_
#define OPENDRIVE_ENGINE_PARAM_H_

#include <memory>
#include <string>

namespace opendrive {
namespace engine {
namespace common {

typedef struct Param ParamType;
struct Param {
  typedef std::shared_ptr<ParamType> Ptr;
  typedef std::shared_ptr<ParamType const> ConstPtr;
  std::string map_file;
  float step = 0;
};

}  // namespace common
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_PARAM_H_
