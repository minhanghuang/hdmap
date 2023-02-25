#ifndef OPENDRIVE_ENGINE_SERVER_PARAM_H_
#define OPENDRIVE_ENGINE_SERVER_PARAM_H_
#include <cactus/cactus.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>

#include "opendrive-engine/common/param.h"

namespace opendrive {
namespace engine {
namespace server {

struct HttpConfig {
  std::string addr;
  int port;
  int thread_num;
};

typedef struct Param ParamType;
class Param {
 public:
  typedef std::shared_ptr<ParamType> Ptr;
  typedef std::shared_ptr<ParamType const> ConstPtr;
  Param() = default;
  int Load(const std::string& yaml_path);
  void Print();
  const HttpConfig& GetHttp() const;
  const common::Param& GetEngine() const;

 private:
  HttpConfig http_;
  common::Param engine_param_;
};

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_PARAM_H_
