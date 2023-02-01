#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

EngineImpl::EngineImpl() {}

int EngineImpl::Init(const common::Param& param) {
  param_ = std::make_shared<common::Param>(param);
  data_ = std::make_shared<core::Data>();
  Clear();
  ConvertData();
  return 0;
}

void EngineImpl::Clear() {}

int EngineImpl::ConvertData() {
  Convertor convertor;
  auto convert_ret = convertor.Start(param_, data_);
  return 0;
}

}  // namespace engine
}  // namespace opendrive
