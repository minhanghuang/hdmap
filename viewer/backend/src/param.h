#ifndef HDMAP_SERVER_PARAM_H_
#define HDMAP_SERVER_PARAM_H_
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>

#include "hdmap/common/utils.h"
#include "hdmap/common/param.h"

namespace hdmap {
namespace server {

struct HttpConfig {
  std::string addr;
  int port;
  int thread_num;
};

class ServerParam {
 public:
  typedef std::shared_ptr<ServerParam> Ptr;
  typedef std::shared_ptr<ServerParam const> ConstPtr;
  ServerParam() = default;
  int Load(const std::string& yaml_path);
  void Print();
  const HttpConfig& http() const;
  const Param& engine_param() const;

 private:
  HttpConfig http_;
  Param engine_param_;
};

}  // namespace server
}  // namespace hdmap

#endif  // HDMAP_SERVER_PARAM_H_
