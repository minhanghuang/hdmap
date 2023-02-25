#include "global_data.h"

#include <memory>

#include "opendrive-engine/common/status.h"

namespace opendrive {
namespace engine {
namespace server {

GlobalData::GlobalData() {}

int GlobalData::Init(const std::string& yaml_path) {
  // param load
  param_ = std::make_shared<Param>();
  if (param_->Load(yaml_path)) {
    return -1;
  }
  param_->Print();

  // engine init
  engine_ = std::make_shared<engine::Engine>();
  auto engine_status = engine_->Init(param_->GetEngine());
  if (ErrorCode::OK != engine_status.error_code) {
    std::cerr << engine_status.msg << std::endl;
    return -1;
  }
  return 0;
}

Param::Ptr GlobalData::GetParam() { return param_; }

engine::Engine::Ptr GlobalData::GetEngine() { return engine_; }

}  // namespace server
}  // namespace engine
}  // namespace opendrive
