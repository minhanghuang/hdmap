#include "api.h"

#include "cyclone/define.h"
#include "opendrive-engine/common/param.h"
#include "opendrive-engine/common/status.h"

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

void OkApi::Get(cyclone::Server* server, cyclone::Connection* conn) {
  Response(server, conn, "ok get");
}

void OkApi::Post(cyclone::Server* server, cyclone::Connection* conn) {
  Response(server, conn, "ok post");
}

void GlobalMapApi::Get(cyclone::Server* server, cyclone::Connection* conn) {
  ELOG_INFO("Http Request GlobalMapApi Get");
  Json response;
  Json line_json;
  for (const auto& lane : engine_->GetLanes()) {
    ConvertLaneToSimplePts(lane, response);
  }
  Response(server, conn, SetResponse(response, HttpStatusCode::SUCCESS, "ok"));
}

void HotUpdate::Post(cyclone::Server* server, cyclone::Connection* conn) {
  ELOG_INFO("Http Request HotUpdate Post");
  Json response;
  std::string req_data = GetRequestData(conn);
  ELOG_INFO("Request Data: " << req_data);
  nlohmann::json data_json;
  if (!CheckRequestData(required_keys_, req_data, data_json)) {
    Response(server, conn,
             SetResponse(nlohmann::json(), HttpStatusCode::PARAM,
                         "Request数据异常"));
    return;
  }
  auto param = common::Param();
  param.map_file = data_json["file"];
  auto status = engine_->HotUpdate(param);
  if (ErrorCode::OK != status.error_code) {
    Response(server, conn,
             SetResponse(response, HttpStatusCode::FAILED, status.msg));
  }
  Response(server, conn, SetResponse(response, HttpStatusCode::SUCCESS, "ok"));
}

void NearestLane::Post(cyclone::Server* server, cyclone::Connection* conn) {
  ELOG_INFO("Http Request NearestLane Post");
  Json response;
  std::string req_data = GetRequestData(conn);
  ELOG_INFO("Request Data: " << req_data);
  nlohmann::json data_json;
  if (!CheckRequestData(required_keys_, req_data, data_json)) {
    Response(server, conn,
             SetResponse(nlohmann::json(), HttpStatusCode::PARAM,
                         "Request数据异常"));
    return;
  }
  auto lanes = engine_->GetNearestLanes(data_json["x"], data_json["y"], 1);
  if (1 != lanes.size()) {
    Response(server, conn,
             SetResponse(nlohmann::json(), HttpStatusCode::FAILED,
                         "Query Nearest Lanes Fault."));
  }
  auto lane = lanes.front();
  ELOG_INFO("Nearest Lane Id: " << lane->id());
  ConvertLaneToSimplePts(lane, response);
  Response(server, conn,
           SetResponse(response, HttpStatusCode::SUCCESS, "get nearest lane"));
}

void Planning::Post(cyclone::Server* server, cyclone::Connection* conn) {
  ELOG_INFO("Http Request Planning Post");
  Json response;
  std::string req_data = GetRequestData(conn);
  ELOG_INFO("Request Data: " << req_data);
  nlohmann::json data_json;
  if (!CheckRequestData(required_keys_, req_data, data_json)) {
    Response(server, conn,
             SetResponse(nlohmann::json(), HttpStatusCode::PARAM,
                         "Request数据异常"));
    return;
  }
  auto points = data_json["points"];
  for (const auto& point : points) {
    if (2 != point.size()) {
      Response(server, conn,
               SetResponse(nlohmann::json(), HttpStatusCode::PARAM,
                           "Request数据异常"));
      return;
    }
    std::cout << "[" << point.front() << "," << point.back() << "]"
              << std::endl;
  }
  Response(server, conn,
           SetResponse(response, HttpStatusCode::SUCCESS, "planning ok"));
}

void RealTimeData::Open(cyclone::Server* server,
                        const cyclone::Connection* conn) {
  ELOG_INFO("WebSocket Request Connection");
}

void RealTimeData::OnMessage(cyclone::Server* server, cyclone::Connection* conn,
                             const std::string& msg, int op_code) {
  // ELOG_INFO("WebSocket OnMessage");
  if (op_code != cyclone::WebSocketOpCode::TEXT) {
    return;
  }
  Json response_json;
  Json request_json;
  if (!CheckRequestData(required_keys_, msg, request_json)) {
    SendData(conn, SetResponse(response_json, HttpStatusCode::PARAM,
                               "WebSocket Data Execption."));
    return;
  }
  double x = request_json["x"];
  double y = request_json["y"];
  auto search = engine_->GetNearestPoints<double>(x, y, 1);
  if (search.empty()) {
    SendData(conn, SetResponse(response_json, HttpStatusCode::PARAM,
                               "WebSocket Data Execption."));
    return;
  }
}

void RealTimeData::OnPong(cyclone::Server* server, cyclone::Connection* conn) {}

void RealTimeData::OnPing(cyclone::Server* server, cyclone::Connection* conn) {}

void RealTimeData::OnClose(cyclone::Server* server,
                           const cyclone::Connection* conn) {}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
