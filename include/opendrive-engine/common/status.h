#ifndef OPENDRIVE_ENGINE_STATUS_H_
#define OPENDRIVE_ENGINE_STATUS_H_

#include <string>

namespace opendrive {
namespace engine {
namespace common {

enum class ErrorCode {
  OK = 0,

  // init
  INIT_ERROR = 1000,
  INIT_MAPFILE_ERROR = 1001,

};

struct StatusBody {
  ErrorCode error_code = ErrorCode::OK;
  std::string msg;
};

class Status {
 public:
  explicit Status(ErrorCode code = ErrorCode::OK, std::string_view msg = "")
      : code_(code), msg_(msg.data()) {}
  ~Status() = default;
  static Status OK() { return Status(); }
  bool ok() const { return code_ == ErrorCode::OK; }
  ErrorCode code() const { return code_; }
  bool operator==(const Status& rh) const {
    return (this->code_ == rh.code_) && (this->msg_ == rh.msg_);
  }
  bool operator!=(const Status& rh) const { return !(*this == rh); }
  const std::string& error_message() const { return msg_; }
  std::string ToString() const {
    if (ok()) {
      return "OK";
    }
    return std::to_string(static_cast<int>(code_)) + ": " + msg_;
  }
  void Save(StatusBody* status_body) {
    if (!status_body) {
      return;
    }
    status_body->error_code = code_;
    if (!msg_.empty()) {
      status_body->msg = msg_;
    }
  }

 private:
  ErrorCode code_;
  std::string msg_;
};

}  // namespace common
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_STATUS_H_
