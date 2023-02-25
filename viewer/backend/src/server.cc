#include "server.h"

namespace opendrive {
namespace engine {

Server::Server() {}

int Server::Init() {
  auto global_data = server::GlobalData::Instance();
  options_.num_threads = global_data->GetParam()->GetHttp().thread_num;
  options_.port = global_data->GetParam()->GetHttp().port;
  options_.root = "";
  ok_ = std::make_shared<server::OkApi>();
  global_map_ = std::make_shared<server::GlobalMapApi>();
  return 0;
}

int Server::Start() {
  typhoon::Server server(options_);
  server.AddHandle("/opendrive/engine/ok/", ok_);
  server.AddHandle("/opendrive/engine/map/", global_map_);
  server.Spin();
  return 0;
}

}  // namespace engine
}  // namespace opendrive
