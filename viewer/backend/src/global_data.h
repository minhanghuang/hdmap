#ifndef OPENDRIVE_ENGINE_SERVER_GLOBAL_DATA_H_
#define OPENDRIVE_ENGINE_SERVER_GLOBAL_DATA_H_
#include <cactus/cactus.h>
#include <cactus/macros.h>
#include <opendrive-engine/engine.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <iostream>

#include "opendrive-engine/common/param.h"
#include "param.h"

namespace opendrive {
namespace engine {
namespace server {

class GlobalData {
 public:
  int Init(const std::string& yaml_path);
  Param::Ptr GetParam();
  engine::Engine::Ptr GetEngine();

 private:
  engine::Engine::Ptr engine_;
  Param::Ptr param_;
  CACTUS_DECLARE_SINGLETON(GlobalData)  // 注册单例
};

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_GLOBAL_DATA_H_
