#ifndef HDMAP_COMMON_RW_LOCK_H_
#define HDMAP_COMMON_RW_LOCK_H_

#include <mutex>
#include <shared_mutex>

namespace hdmap {
namespace common {

std::unique_lock<std::shared_mutex> WriteLock(std::shared_mutex& mutex);

std::unique_lock<std::shared_mutex> WriteLock(std::shared_mutex* mutex);

std::shared_lock<std::shared_mutex> ReadLock(std::shared_mutex& mutex);

std::shared_lock<std::shared_mutex> ReadLock(std::shared_mutex* mutex);

}  // namespace common
}  // namespace hdmap

#endif  // HDMAP_COMMON_RW_LOCK_H_
