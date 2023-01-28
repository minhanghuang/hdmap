#ifndef OPENDRIVE_ENGINE_IMPL_H_
#define OPENDRIVE_ENGINE_IMPL_H_

#include <memory>
#include <string>

#include "opendrive-engine/common/param.h"

namespace opendrive {
namespace engine {

typedef class EngineImpl EngineImplType;
class EngineImpl {
 public:
  typedef std::shared_ptr<EngineImplType> Ptr;
  explicit EngineImpl(common::Param::ConstPtr param);
  explicit EngineImpl(const common::Param& param);

 private:
  void Clear();
  common::Param::ConstPtr param_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_IMPL_H_
