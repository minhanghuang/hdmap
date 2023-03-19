#ifndef OPENDRIVE_ENGINE_SERVER_H_
#define OPENDRIVE_ENGINE_SERVER_H_
#include <cyclone/cyclone.h>
#include <cyclone/options.h>

#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "api.h"
#include "global_data.h"

namespace opendrive {
namespace engine {

class Server {
 public:
  ~Server() = default;
  Server();
  int Init();
  int Start();

 private:
  cyclone::Options options_;
  std::shared_ptr<server::OkApi> ok_;
  std::shared_ptr<server::GlobalMapApi> global_map_;
  std::shared_ptr<server::NearestLane> nearest_lane_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_H_
