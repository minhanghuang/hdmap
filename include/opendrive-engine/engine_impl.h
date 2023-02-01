#ifndef OPENDRIVE_ENGINE_IMPL_H_
#define OPENDRIVE_ENGINE_IMPL_H_

#include <memory>
#include <string>

#include "opendrive-engine/common/param.h"
#include "opendrive-engine/convertor.h"
#include "opendrive-engine/core/define.h"
#include "opendrive-engine/core/map.h"

namespace opendrive {
namespace engine {

typedef class EngineImpl EngineImplType;
class EngineImpl {
 public:
  typedef std::shared_ptr<EngineImplType> Ptr;
  EngineImpl();
  int Init(const common::Param& param);

 private:
  void Clear();
  int ConvertData();
  common::Param::ConstPtr param_;
  core::Data::Ptr data_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_IMPL_H_
