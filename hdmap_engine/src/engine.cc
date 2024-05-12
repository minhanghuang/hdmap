#include "hdmap_engine/engine.h"

namespace hdmap {

Engine::Engine() : impl_(std::make_shared<EngineImpl>()) {}

bool Engine::Init(const Param& param) { return impl_->Init(param); }

bool Engine::HotUpdate(const Param& param) { return impl_->Init(param); }

bool Engine::GetPointById(const std::string& point_id,
                          geometry::Curve::Point& out_point) {
  return impl_->GetPointById(point_id, out_point);
}

geometry::Lane::ConstPtr Engine::GetLaneById(const std::string& id) {
  return impl_->GetLaneById(id);
}

geometry::Section::ConstPtr Engine::GetSectionById(const std::string& id) {
  return impl_->GetSectionById(id);
}

geometry::Road::ConstPtr Engine::GetRoadById(const std::string& id) {
  return impl_->GetRoadById(id);
}

geometry::Lane::ConstPtrs Engine::GetLanes() { return impl_->GetLanes(); }

geometry::Section::ConstPtrs Engine::GetSections() {
  return impl_->GetSections();
}

geometry::Road::ConstPtrs Engine::GetRoads() { return impl_->GetRoads(); }

geometry::Header::ConstPtr Engine::GetHeader() { return impl_->GetHeader(); }

}  // namespace hdmap
