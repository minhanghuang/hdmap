#ifndef HDMAP_SERVER_H_
#define HDMAP_SERVER_H_
#include <cyclone/cyclone.h>
#include <cyclone/options.h>

#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "api.h"
#include "global_data.h"

namespace hdmap {

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
  std::shared_ptr<server::HotUpdate> hot_update_;
  std::shared_ptr<server::Planning> planning_;
  std::shared_ptr<server::RealTimeData> real_time_data_;
};

}  // namespace hdmap

#endif  // HDMAP_SERVER_H_
