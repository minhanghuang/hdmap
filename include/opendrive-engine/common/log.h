#ifndef OPENDRIVE_ENGINE_LOG_H_
#define OPENDRIVE_ENGINE_LOG_H_

#include <iostream>

namespace opendrive {
namespace engine {

#define ENGINE_INFO(...) \
  std::cout << "--- engine log: " << __VA_ARGS__ << std::endl;

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_LOG_H_
