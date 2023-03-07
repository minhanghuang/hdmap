#ifndef OPENDRIVE_ENGINE_IMPL_H_
#define OPENDRIVE_ENGINE_IMPL_H_

#include <cactus/cactus.h>
#include <cactus/factory.h>

#include <memory>
#include <string>

#include "opendrive-cpp/common/status.h"
#include "opendrive-engine/common/param.h"
#include "opendrive-engine/convertor.h"
#include "opendrive-engine/core/define.h"
#include "opendrive-engine/core/header.h"

namespace opendrive {
namespace engine {

class EngineImpl {
 public:
  typedef std::shared_ptr<EngineImpl> Ptr;
  EngineImpl();
  Status Init(const common::Param& param);
  std::string GetXodrVersion() const;
  core::Lane::ConstPtr GetLaneById(const core::Id& id) const;
  core::Section::ConstPtr GetSectionById(const core::Id& id) const;
  core::Road::ConstPtr GetRoadById(const core::Id& id) const;
  core::Lane::ConstPtrs GetLanes() const;
  core::Section::ConstPtrs GetSections() const;
  core::Road::ConstPtrs GetRoads() const;
  core::Header::ConstPtr GetHeader() const;

 private:
  core::Data::Ptr data_;
  common::Param::ConstPtr param_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_IMPL_H_
