#include "api.h"

#include "global_data.h"

namespace opendrive {
namespace engine {
namespace server {

bool RequestBase::CheckRequestData(
    const std::unordered_map<std::string, nlohmann::json::value_t>& keys,
    const std::string& data, Json& data_out) {
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
  return true;
}

std::string RequestBase::SetResponse(const Json& data, HttpStatusCode code,
                                     const std::string& msg) {
  Json response;
  response["code"] = code;
  response["msg"] = msg;
  response["results"] = data;
  return response.dump(0);
}

void OkApi::Get(typhoon::Application* app, typhoon::Connection* conn) {
  Response(app, conn, "ok get");
}

void OkApi::Post(typhoon::Application* app, typhoon::Connection* conn) {
  Response(app, conn, "ok post");
}

void GlobalMapApi::Get(typhoon::Application* app, typhoon::Connection* conn) {
  auto engine = GlobalData::Instance()->GetEngine();
  for (const auto& lane : engine->GetLanes()) {
    std::cout << "lane: " << lane->id() << std::endl;
  }
  Response(app, conn, "ok post");
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
