#ifndef HDMAP_SERVER_GLOBAL_DATA_H_
#define HDMAP_SERVER_GLOBAL_DATA_H_
#include <hdmap/engine.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <iostream>

#include "hdmap/common/macros.h"
#include "hdmap/common/param.h"
#include "param.h"

namespace hdmap {
namespace server {

class GlobalData {
 public:
  int Init(const std::string& yaml_path);
  ServerParam::Ptr GetParam();
  Engine::Ptr GetEngine();

 private:
  Engine::Ptr engine_;
  ServerParam::Ptr param_;
  HDMAP_DECLARE_SINGLETON(GlobalData)  // 注册单例
};

}  // namespace server
}  // namespace hdmap

#endif  // HDMAP_SERVER_GLOBAL_DATA_H_
