#ifndef OPENDRIVE_ENGINE_SERVER_API_H_
#define OPENDRIVE_ENGINE_SERVER_API_H_
#include <cyclone/cyclone.h>

#include <mutex>
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
  virtual void Get(cyclone::Server* server, cyclone::Connection* conn) override;
  virtual void Post(cyclone::Server* server,
                    cyclone::Connection* conn) override;
};

class GlobalMapApi : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Get(cyclone::Server* server, cyclone::Connection* conn) override;
};

class HotUpdate : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Post(cyclone::Server* server,
                    cyclone::Connection* conn) override;

 private:
  RequiredKeys required_keys_{
      std::make_pair("file", JsonValueType::string),
  };
};

class NearestLane : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Post(cyclone::Server* server,
                    cyclone::Connection* conn) override;

 private:
  RequiredKeys required_keys_{
      std::make_pair("x", JsonValueType::number_float),
      std::make_pair("y", JsonValueType::number_float),
  };
};

class Planning : public cyclone::web::RequestHandler, public RequestBase {
 public:
  virtual void Post(cyclone::Server* server,
                    cyclone::Connection* conn) override;

 private:
  RequiredKeys required_keys_{
      std::make_pair("points", JsonValueType::array),
  };
};

class RealTimeData : public cyclone::websocket::WebSocketHandler,
                     public RequestBase {
 public:
  virtual void Open(cyclone::Server* server,
                    const cyclone::Connection* conn) override;
  virtual void OnMessage(cyclone::Server* server, cyclone::Connection* conn,
                         const std::string& msg, int op_code) override;
  virtual void OnPong(cyclone::Server* server,
                      cyclone::Connection* conn) override;
  virtual void OnPing(cyclone::Server* server,
                      cyclone::Connection* conn) override;
  virtual void OnClose(cyclone::Server* server,
                       const cyclone::Connection* conn) override;

 private:
  RequiredKeys required_keys_{
      std::make_pair("x", JsonValueType::number_float),
      std::make_pair("y", JsonValueType::number_float),
  };
  // std::mutex mutex_;
};

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_API_H_
