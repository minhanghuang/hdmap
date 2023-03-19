#include "server.h"

namespace opendrive {
namespace engine {

Server::Server() {}

int Server::Init() {
  auto global_data = server::GlobalData::Instance();
  options_.num_threads = global_data->GetParam()->http().thread_num;
  options_.port = global_data->GetParam()->http().port;
  options_.root = "";
  ok_ = std::make_shared<server::OkApi>();
  global_map_ = std::make_shared<server::GlobalMapApi>();
  nearest_lane_ = std::make_shared<server::NearestLane>();
  return 0;
}

int Server::Start() {
  cyclone::web::Application app;
  app.Init(options_);
  app.AddHandler("/opendrive/engine/ok/", ok_);
  app.AddHandler("/opendrive/engine/map/", global_map_);
  app.AddHandler("/opendrive/engine/nearest_lane/", nearest_lane_);
  app.Spin();
  return 0;
}

}  // namespace engine
}  // namespace opendrive
