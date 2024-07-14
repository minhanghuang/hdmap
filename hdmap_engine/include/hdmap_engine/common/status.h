#ifndef HDMAP_ENGINE_STATUS_H_
#define HDMAP_ENGINE_STATUS_H_

#include <string>

#include "hdmap_engine/common/macros.h"

namespace hdmap {

enum class ErrorCode {
  kOk = 0,

  /// xml
  XML_ROOT_ELEMENT_ERROR = 1000,
  XML_HEADER_ELEMENT_ERROR,
  XML_ROAD_ELEMENT_ERROR,
  XML_ROAD_LINK_ELEMENT_ERROR,
  XML_ROAD_PLANVIEW_ELEMENT_ERROR,
  XML_ROAD_TYPE_ELEMENT_ERROR,
  XML_LANES_ELEMENT_ERROR,
  XML_LANES_OFFSET_ELEMENT_ERROR,
  XML_LANES_SECTION_ELEMENT_ERROR,
  XML_JUNCTION_ELEMENT_ERROR,
  XML_JUNCTION_CONNECTION_ELEMENT_ERROR,

  /// adapter
  ADAPTER_ROOT_ERROR = 2000,
  ADAPTER_HEADER_ERROR,
  ADAPTER_JUNCTION_ERROR,
  ADAPTER_ROAD_ERROR,
  ADAPTER_LINE_ERROR,
  ADAPTER_LANE_ERROR,
  ADAPTER_SECTION_ERROR,
  ADAPTER_GEOMETRY_ERROR,
  ADAPTER_ROADTYPE_ERROR,

  // init
  kInitError = 3000,
  kInitFactoryError,
  kInitMapfileError,

  // convert
  kConvertorError = 4000,
  kConvertorXmlParserError,
  kConvertorCenterlaneError,

  // routing
  kRoutingError = 5000,
};

class Status {
 public:
  using Ptr = std::shared_ptr<Status>;
  using ConstPtr = std::shared_ptr<Status const>;
  Status();

  /**
   * @brief Get error code
   *
   * @return
   */
  ErrorCode error_code() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return error_code_;
  }
  /**
   * @brief Get msg
   *
   * @return
   */
  std::string msg() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return msg_;
  }

  /**
   * @brief statue is ok
   *
   * @return
   */
  bool ok() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return ErrorCode::kOk == error_code_;
  }

  /**
   * @brief Set error code
   *
   * @param error_code
   */
  void set_error_code(ErrorCode error_code) {
    std::lock_guard<std::mutex> guard(mutex_);
    error_code_ = error_code;
  }
  /**
   * @brief Set msg
   *
   * @param msg
   */
  void set_msg(const std::string& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    msg_ = msg;
  }

  /**
   * @brief Set status
   *
   * @param error_code
   * @param msg
   */
  void set_status(ErrorCode error_code, const std::string& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    error_code_ = error_code;
    msg_ = msg;
  }

  void set_status(const Status& status) {
    std::lock_guard<std::mutex> guard(mutex_);
    error_code_ = status.error_code();
    msg_ = status.msg();
  }

 private:
  mutable std::mutex mutex_;

  ErrorCode error_code_ = ErrorCode::kOk;

  std::string msg_ = "ok";
};

}  // namespace hdmap

#endif  // HDMAP_ENGINE_STATUS_H_
