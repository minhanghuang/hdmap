#ifndef OPENDRIVE_ENGINE_SERVER_LOG_HPP_
#define OPENDRIVE_ENGINE_SERVER_LOG_HPP_
#include <glog/logging.h>
#include <glog/stl_logging.h>

#include <chrono>
#include <iostream>

namespace opendrive {
namespace engine {
namespace server {

static void LogInit(const std::string& log_path) {
  FLAGS_log_dir = log_path;
  google::InitGoogleLogging("engine_server");
  google::SetStderrLogging(google::GLOG_INFO);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = false;
}

#define ENGINE_SERVER_INFO(...) LOG(INFO) << __VA_ARGS__;
#define ENGINE_SERVER_WARN(...) LOG(WARNING) << __VA_ARGS__;
#define ENGINE_SERVER_ERROR(...) LOG(ERROR) << __VA_ARGS__;

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_LOG_HPP_
