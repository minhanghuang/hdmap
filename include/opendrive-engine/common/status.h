#ifndef OPENDRIVE_ENGINE_STATUS_H_
#define OPENDRIVE_ENGINE_STATUS_H_

#include <string>

namespace opendrive {
namespace engine {

enum class ErrorCode {
  OK = 0,

  // init
  kInitError = 1000,
  kInitFactoryError,
  kInitMapfileError,

  // convert
  kConvertorError = 2000,
  kConvertorXmlParserError,
  kConvertorCenterlaneError,

  // routing
  kRoutingError = 3000,
};

struct Status {
  Status() = default;
  explicit Status(ErrorCode e) : error_code(e) {}
  Status(ErrorCode e, const std::string& m) : error_code(e), msg(m) {}
  ErrorCode error_code = ErrorCode::OK;
  std::string msg;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_STATUS_H_
