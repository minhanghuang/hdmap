#include "global_data.h"

namespace opendrive {
namespace engine {
namespace server {

GlobalData* GlobalData::Instance() {
  static GlobalData* instance = nullptr;
  if (!instance) {
    static std::once_flag flag;
    std::call_once(flag, [&] { instance = new (std::nothrow) GlobalData(); });
  }
  return instance;
}

void GlobalData::Init(const std::string& yaml_file) {
  yaml_file_ = yaml_file;
  auto param = std::make_shared<engine::common::Param>();

  /// parse yaml
  auto node = YAML::LoadFile(yaml_file_);
  std::string map_file = node["opendrive_file"].as<std::string>();
  log_path_ = node["log_path"].as<std::string>();
  server_port_ = node["server_port"].as<int>();
  server_thread_num_ = node["server_thread_num"].as<int>();
  if (map_file.empty()) {
    throw std::runtime_error("opendrive file is empty." + map_file);
  }
  if (server_port_ <= 0 || server_thread_num_ <= 0) {
    throw std::runtime_error("server param error.");
  }

  /// instance
  param->map_file = map_file;
  engine_ = std::make_shared<engine::Engine>(param);
}

std::string GlobalData::log_path() { return log_path_; }

size_t GlobalData::server_port() { return server_port_; }

unsigned char GlobalData::server_thread_num() { return server_thread_num_; }

engine::Engine::Ptr GlobalData::engine() { return engine_; }

}  // namespace server
}  // namespace engine
}  // namespace opendrive
