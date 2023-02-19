#include "opendrive-engine/engine.h"

namespace opendrive {
namespace engine {

Engine::Engine() : impl_(std::make_shared<EngineImpl>()) {}

Status Engine::Init(const common::Param& param) {
  cactus::WriteLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->Init(param);
}

std::string Engine::GetXodrVersion() {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetXodrVersion();
}

}  // namespace engine
}  // namespace opendrive
