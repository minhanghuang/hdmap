#ifndef OPENDRIVE_ENGINE_SERVER_API_H_
#define OPENDRIVE_ENGINE_SERVER_API_H_
#include <cyclone/cyclone.h>

#include <nlohmann/json.hpp>

#include "cyclone/define.h"
#include "global_data.h"
#include "log.h"
#include "util.h"

namespace opendrive {
namespace engine {
namespace server {

enum class HttpStatusCode {
  SUCCESS = 1000,
  FAILED,
  PARAM,
};

class RequestBase {
 public:
  RequestBase();
  virtual ~RequestBase() = default;

 protected:
  virtual bool CheckRequestData(const RequiredKeys& keys,
                                const std::string& data, Json& data_out) final;
  virtual bool IsNumberType(const Json& value) const final;
  virtual std::string SetResponse(const Json& data,
                                  HttpStatusCode code = HttpStatusCode::SUCCESS,
                                  const std::string& msg = "ok") final;
  engine::Engine::Ptr engine_;
};

class OkApi : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Get() override;
  virtual void Post() override;
};

class GlobalMapApi : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Get() override;
};

class NearestLane : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Post() override;

 private:
  RequiredKeys required_keys_{
      std::make_pair("x", nlohmann::json::value_t::number_float),
      std::make_pair("y", nlohmann::json::value_t::number_float),
  };
};

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_API_H_
