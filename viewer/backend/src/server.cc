#include "server.h"

namespace opendrive {
namespace engine {
namespace server {

bool RequestBase::CheckRequestData(const RequiredKeys& keys,
                                   const std::string& data,
                                   Json& data_out) const {
  if (data.empty()) return false;
  try {
    data_out = Json::parse(data);
  } catch (const std::exception& e) {
    return false;
  }
  for (const auto& item : keys) {
    if (!data_out.contains(item.first)) {
      return false;
    }
    if (data_out[item.first].type() != item.second) {
      if (item.second == JsonValueType::number_float ||
          item.second == JsonValueType::number_integer ||
          item.second == JsonValueType::number_unsigned) {
        if (IsNumberType(data_out[item.first])) {
          continue;
        }
      }
      return false;
    }
  }
  return true;
}

bool RequestBase::IsNumberType(const Json& value) const {
  if (value.is_number_float() || value.is_number_integer() ||
      value.is_number_unsigned()) {
    return true;
  }
  return false;
}

std::string RequestBase::SetResponse(const Json& data, HttpStatusCode code,
                                     const std::string& msg) {
  Json response;
  response["code"] = code;
  response["msg"] = msg;
  response["results"] = data;
  return response.dump(0);
}

void OKApi::Get(typhoon::Application* app, typhoon::Connection* conn) {
  ENGINE_SERVER_INFO("HDMap Server OkApi Get Requets.");
  Response(app, conn, "ok get");
}

void OKApi::Post(typhoon::Application* app, typhoon::Connection* conn) {
  ENGINE_SERVER_INFO("HDMap Server OkApi Post Requets.");
  Response(app, conn, "ok post");
}

void Server::Init(const std::string& yaml_file) {
  assert(!yaml_file.empty());
  GlobalData::Instance()->Init(yaml_file);
  server::LogInit(server::GlobalData::Instance()->log_path());
  ENGINE_SERVER_INFO("hdmap server yaml file:" << yaml_file);
  ENGINE_SERVER_INFO(
      "hdmap server log path: " << server::GlobalData::Instance()->log_path());
}

void Server::Start() {
  typhoon::Options options;
  options.port = server::GlobalData::Instance()->server_port();
  options.num_threads = server::GlobalData::Instance()->server_thread_num();
  options.root = "";
  ENGINE_SERVER_INFO("hdmap server port: " << options.port);
  ENGINE_SERVER_INFO("hdmap server threads: " << options.num_threads);
  typhoon::Server server(options);
  auto ok = std::make_shared<OKApi>();
  server.Spin();
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
