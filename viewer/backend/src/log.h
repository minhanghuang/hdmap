#ifndef OPENDRIVE_ENGINE_SERVER_LOG_H_
#define OPENDRIVE_ENGINE_SERVER_LOG_H_

#include <iostream>

namespace opendrive {
namespace engine {
namespace server {

#define ELOG_INFO(...)                                                 \
  std::cout << "[xodr engine server] [info] [" << __FILE_NAME__ << ":" \
            << __LINE__ << "] " << __VA_ARGS__ << std::endl;

}  // namespace server
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_SERVER_LOG_H_
