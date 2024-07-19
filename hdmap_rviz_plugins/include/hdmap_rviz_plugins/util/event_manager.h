#ifndef HDMAP_RVIZ_PLUGIN_UTIL_EVENT_MANAGER_H_
#define HDMAP_RVIZ_PLUGIN_UTIL_EVENT_MANAGER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "hdmap_common/macros.h"
#include "hdmap_common/rw_lock.h"

namespace hdmap_rviz_plugins {

class EventManager {
 public:
  enum class EventType {
    kMouseCursorEvent = 0,
    kSelectFileEvent,
  };

  void RegisterCallback(EventType event_type,
                        std::function<void(void*)> callback);

  void TriggerEvent(EventType event_type, void* data);

 private:
  std::shared_mutex mutex_;
  std::unordered_map<EventType, std::vector<std::function<void(void*)>>>
      callbacks_;

  HDMAP_DECLARE_SINGLETON(EventManager)
};

}  // namespace hdmap_rviz_plugins

#endif  // HDMAP_RVIZ_PLUGIN_UTIL_EVENT_MANAGER_H_
