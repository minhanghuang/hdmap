#ifndef OPENDRIVE_ENGINE_H_
#define OPENDRIVE_ENGINE_H_

#include <memory>
#include <string>

#include "opendrive-engine/common/param.h"
#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

typedef class Engine EngineType;
class Engine {
 public:
  typedef std::shared_ptr<EngineType> Ptr;
  typedef std::unique_ptr<EngineType> UPtr;
  typedef std::unique_ptr<EngineType const> ConstPtr;
  ~Engine() = default;
  Engine();
  int Init(const common::Param& param);

 private:
  EngineImpl::Ptr impl_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_H_
