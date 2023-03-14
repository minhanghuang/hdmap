#include "api.h"

#include "global_data.h"
#include "log.h"
#include "util.h"

namespace opendrive {
namespace engine {
namespace server {

RequestBase::RequestBase() { engine_ = GlobalData::Instance()->GetEngine(); }

bool RequestBase::CheckRequestData(const RequiredKeys& keys,
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

void OkApi::Get(typhoon::Application* app, typhoon::Connection* conn) {
  Response(app, conn, "ok get");
}

void OkApi::Post(typhoon::Application* app, typhoon::Connection* conn) {
  Response(app, conn, "ok post");
}

void GlobalMapApi::Get(typhoon::Application* app, typhoon::Connection* conn) {
  ELOG_INFO("Http Request GlobalMapApi Get");
  Json response;
  Json line_json;
  for (const auto& lane : engine_->GetLanes()) {
    ConvertLaneToSimplePts(lane, response);
  }
  Response(app, conn, SetResponse(response, HttpStatusCode::SUCCESS, "ok"));
}

void NearestLane::Post(typhoon::Application* app, typhoon::Connection* conn) {
  ELOG_INFO("Http Request NearestLane Post");
  Json response;
  std::string req_data = typhoon::RequestHandler::GetRequestData(conn);
  ELOG_INFO("Request Data: " << req_data);
  nlohmann::json data_json;
  if (!CheckRequestData(required_keys_, req_data, data_json)) {
    typhoon::RequestHandler::Response(
        app, conn,
        SetResponse(nlohmann::json(), HttpStatusCode::PARAM,
                    "Request数据异常"));
    return;
  }
  auto lanes = engine_->GetNearestLanes(data_json["x"], data_json["y"], 1);
  if (1 != lanes.size()) {
    typhoon::RequestHandler::Response(
        app, conn,
        SetResponse(nlohmann::json(), HttpStatusCode::FAILED,
                    "Query Nearest Lanes Fault."));
  }
  auto lane = lanes.front();
  ELOG_INFO("Nearest Lane Id: " << lane->id());
  ConvertLaneToSimplePts(lane, response);
  Response(app, conn,
           SetResponse(response, HttpStatusCode::SUCCESS, "get nearest lane"));
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
