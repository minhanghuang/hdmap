#include "global_data.h"

namespace hdmap {
namespace server {

GlobalData::GlobalData() {}

int GlobalData::Init(const std::string& yaml_path) {
  std::cout << "GlobalData Init Start." << std::endl;
  // param load
  param_ = std::make_shared<ServerParam>();
  if (param_->Load(yaml_path)) {
    return -1;
  }
  param_->Print();

  // engine init
  engine_ = std::make_shared<Engine>();
  if (!engine_->Init(param_->engine_param())) {
    std::cerr << "engine init exception: " << engine_->status()->msg()
              << std::endl;
    return -1;
  }
  std::cout << "GlobalData Init End." << std::endl;
  return 0;
}

ServerParam::Ptr GlobalData::GetParam() { return param_; }

Engine::Ptr GlobalData::GetEngine() { return engine_; }

}  // namespace server
}  // namespace hdmap
