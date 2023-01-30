#ifndef OPENDRIVE_ENGINE_SERVER_H_
#define OPENDRIVE_ENGINE_SERVER_H_

#include <opendrive-engine/common/param.h>
#include <typhoon/civethandle.h>
#include <typhoon/typhoon.h>

#include <nlohmann/json.hpp>
#include <stdexcept>

#include "global_data.h"
#include "log.hpp"
#include "opendrive-engine/engine.h"

namespace opendrive {
namespace engine {
namespace server {

enum class HttpStatusCode : std::size_t {
  OK = 1000,
  ERROR,
  PARAM,
};

typedef nlohmann::json Json;
typedef nlohmann::json::value_t JsonValueType;
typedef std::string Data;
typedef std::unordered_map<std::string, JsonValueType> RequiredKeys;

class RequestBase {
 public:
  virtual ~RequestBase() = default;
  virtual bool CheckRequestData(const RequiredKeys& keys,
                                const std::string& data,
                                Json& data_out) const final;
  virtual bool IsNumberType(const Json& value) const final;
  virtual std::string SetResponse(const Json& data,
                                  HttpStatusCode code = HttpStatusCode::OK,
                                  const std::string& msg = "ok") final;
};

class OKApi : public typhoon::RequestHandler, public RequestBase {
 public:
  virtual void Get(typhoon::Application* app,
                   typhoon::Connection* conn) override;
  virtual void Post(typhoon::Application* app,
                    typhoon::Connection* conn) override;
};

class Server {
 public:
  ~Server() = default;
  Server() = default;
  void Init(const std::string& yaml_file);
  void Start();
};

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_H_
