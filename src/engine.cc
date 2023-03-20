#include "opendrive-engine/engine.h"

namespace opendrive {
namespace engine {

Engine::Engine() : impl_(std::make_shared<EngineImpl>()) {}

Status Engine::Init(const common::Param& param) {
  cactus::WriteLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->Init(param);
}

Status Engine::HotUpdate(const common::Param& param) {
  cactus::WriteLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->Init(param);
}

std::string Engine::GetXodrVersion() {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetXodrVersion();
}

bool Engine::GetPointById(const core::Id& point_id,
                          core::Curve::Point& out_point) {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetPointById(point_id, out_point);
}

core::Lane::ConstPtr Engine::GetLaneById(const core::Id& id) {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetLaneById(id);
}

core::Section::ConstPtr Engine::GetSectionById(const core::Id& id) {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetSectionById(id);
}

core::Road::ConstPtr Engine::GetRoadById(const core::Id& id) {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetRoadById(id);
}

core::Lane::ConstPtrs Engine::GetLanes() {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetLanes();
}

core::Section::ConstPtrs Engine::GetSections() {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetSections();
}

core::Road::ConstPtrs Engine::GetRoads() {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetRoads();
}

core::Header::ConstPtr Engine::GetHeader() {
  cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
  return impl_->GetHeader();
}

}  // namespace engine
}  // namespace opendrive
