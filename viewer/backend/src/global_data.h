#ifndef OPENDRIVE_ENGINE_SERVER_GLOBAL_DATA_H_
#define OPENDRIVE_ENGINE_SERVER_GLOBAL_DATA_H_

#include <opendrive-engine/common/param.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

#include "opendrive-engine/engine.h"

namespace opendrive {
namespace engine {
namespace server {

class GlobalData {
 public:
  static GlobalData* Instance();
  void Init(const std::string& yaml_file);
  std::string log_path();
  size_t server_port();
  unsigned char server_thread_num();
  engine::Engine::Ptr engine();

 private:
  GlobalData() = default;
  ~GlobalData() = default;
  GlobalData(const GlobalData&) = delete;
  GlobalData& operator=(const GlobalData&) = delete;
  std::string yaml_file_;
  std::string log_path_;
  size_t server_port_;
  unsigned char server_thread_num_;
  engine::Engine::Ptr engine_;
};

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_GLOBAL_DATA_H_
