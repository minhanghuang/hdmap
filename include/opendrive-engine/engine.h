#ifndef OPENDRIVE_ENGINE_H_
#define OPENDRIVE_ENGINE_H_

#include <memory>
#include <string>

#include "opendrive-engine/common/param.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/id.h"
#include "opendrive-engine/core/lane.h"
#include "opendrive-engine/core/road.h"
#include "opendrive-engine/core/section.h"
#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

class Engine {
 public:
  typedef std::shared_ptr<Engine> Ptr;
  typedef std::unique_ptr<Engine> UPtr;
  typedef std::shared_ptr<Engine const> ConstPtr;
  ~Engine() = default;
  Engine();
  Status Init(const common::Param& param);
  std::string GetXodrVersion();
  core::Curve::Point GetPointById(const core::Id& id);
  core::Lane::ConstPtr GetLaneById(const core::Id& id);
  core::Section::ConstPtr GetSectionById(const core::Id& id);
  core::Road::ConstPtr GetRoadById(const core::Id& id);
  core::Lane::ConstPtrs GetLanes();
  core::Section::ConstPtrs GetSections();
  core::Road::ConstPtrs GetRoads();
  core::Header::ConstPtr GetHeader();
  template <typename T>
  kdtree::KDTreeResults GetNearestPoints(T x, T y, size_t num_closest) {
    return impl_->GetNearestPoints(static_cast<double>(x),
                                   static_cast<double>(y), num_closest);
  }
  template <typename T>
  kdtree::KDTreeResults GetNearestPoints(const T& query_point,
                                         size_t num_closest) {
    return impl_->GetNearestPoints(query_point.x(), query_point.y(),
                                   num_closest);
  }

 private:
  EngineImpl::Ptr impl_;
  cactus::AtomicRWLock rw_lock_;  // read and write lock
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_H_
