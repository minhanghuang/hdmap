#include "hdmap_common/rw_lock.h"

namespace hdmap {
namespace common {

std::unique_lock<std::shared_mutex> WriteLock(std::shared_mutex& mutex) {
  return std::unique_lock<std::shared_mutex>(mutex);
}

std::unique_lock<std::shared_mutex> WriteLock(std::shared_mutex* mutex) {
  return std::unique_lock<std::shared_mutex>(*mutex);
}

std::shared_lock<std::shared_mutex> ReadLock(std::shared_mutex& mutex) {
  return std::shared_lock<std::shared_mutex>(mutex);
}

std::shared_lock<std::shared_mutex> ReadLock(std::shared_mutex* mutex) {
  return std::shared_lock<std::shared_mutex>(*mutex);
}

}  // namespace common
}  // namespace hdmap
