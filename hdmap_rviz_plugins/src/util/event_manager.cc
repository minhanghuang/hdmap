#include "util/event_manager.h"

namespace hdmap_rviz_plugins {

EventManager::EventManager() {}

void EventManager::RegisterCallback(EventType event_type,
                                    std::function<void(void*)> callback) {
  hdmap::common::WriteLock(mutex_);
  callbacks_[event_type].emplace_back(callback);
}

void EventManager::TriggerEvent(EventType event_type, void* data) {
  hdmap::common::ReadLock(mutex_);
  for (const auto& callback : callbacks_[event_type]) {
    callback(data);
  }
}

}  // namespace hdmap_rviz_plugins
