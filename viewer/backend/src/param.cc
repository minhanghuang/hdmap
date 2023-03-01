#include "param.h"

#include <algorithm>

#include "cactus/cactus.h"

namespace opendrive {
namespace engine {
namespace server {

int Param::Load(const std::string& yaml_path) {
  if (!cactus::FileExists(yaml_path)) {
    return -1;
  }
  auto yaml_node = YAML::LoadFile(yaml_path);
  for (auto foo : yaml_node["engine"]) {
    std::string key = foo.first.as<std::string>();
    if ("opendrive_file" == key) {
      engine_param_.map_file = foo.second.as<std::string>();
    } else if ("step" == key) {
      engine_param_.step = foo.second.as<float>();
    }
  }
  for (auto foo : yaml_node["http"]) {
    std::string key = foo.first.as<std::string>();
    if ("addr" == key) {
      http_.addr = foo.second.as<std::string>();
    } else if ("port" == key) {
      http_.port = std::max(1, foo.second.as<int>());
    } else if ("thread_num" == key) {
      http_.thread_num = std::max(6, foo.second.as<int>());
    }
  }
  return 0;
}

void Param::Print() {
  std::cout << "param engine file: " << engine_param_.map_file << std::endl;
  std::cout << "param engine step: " << engine_param_.step << std::endl;
  std::cout << "param http addr: " << http_.addr << std::endl;
  std::cout << "param http port: " << http_.port << std::endl;
  std::cout << "param http thread_num: " << http_.thread_num << std::endl;
}

const HttpConfig& Param::http() const { return http_; }

const common::Param& Param::engine_param() const { return engine_param_; }

}  // namespace server
}  // namespace engine
}  // namespace opendrive
