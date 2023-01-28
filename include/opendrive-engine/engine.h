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
  ~Engine();
  explicit Engine(const common::Param& param);
  explicit Engine(common::Param::Ptr param);

 private:
  void Init();
  EngineImpl::Ptr impl_;
};
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_H_
