#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

EngineImpl::EngineImpl()
    : param_(std::make_shared<common::Param>()),
      data_(std::make_shared<core::Data>()) {}

Status EngineImpl::Init(const common::Param& param) {
  // factory load
  auto factory = cactus::Factory::Instance();
  factory->Register<common::Param>(&param, "engine_param", true);
  factory->Register<core::Data>("core_data", true);
  param_ = factory->GetObject<common::Param>("engine_param");
  data_ = factory->GetObject<core::Data>("core_data");

  // convert data
  Convertor convertor;
  return convertor.Start();
}

std::string EngineImpl::GetXodrVersion() const {
  return data_->header->rev_major + "." + data_->header->rev_minor + "." +
         data_->header->version;
}

}  // namespace engine
}  // namespace opendrive
