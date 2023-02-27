#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

EngineImpl::EngineImpl()
    : param_(std::make_shared<common::Param>()),
      data_(std::make_shared<core::Data>()) {}

Status EngineImpl::Init(const common::Param& param) {
  // factory load
  auto factory = cactus::Factory::Instance();
  factory->Register<common::Param>(&param, "engine_param", true);
  factory->Register<core::Data>("core_data", true);
  param_ = factory->GetObject<common::Param>("engine_param");
  data_ = factory->GetObject<core::Data>("core_data");
  ENGINE_INFO("Factory Load End.");

  // convert data
  Convertor convertor;
  return convertor.Start();
}

std::string EngineImpl::GetXodrVersion() const {
  return data_->header->rev_major() + "." + data_->header->rev_minor() + "." +
         data_->header->version();
}

core::Lane::ConstPtr EngineImpl::GetLaneById(const core::Id& id) const {
  if (data_->lanes.count(id)) {
    return data_->lanes.at(id);
  }
  return nullptr;
}

core::Section::ConstPtr EngineImpl::GetSectionById(const core::Id& id) const {
  if (data_->sections.count(id)) {
    return data_->sections.at(id);
  }
  return nullptr;
}

core::Road::ConstPtr EngineImpl::GetRoadById(const core::Id& id) const {
  if (data_->roads.count(id)) {
    return data_->roads.at(id);
  }
  return nullptr;
}

core::Lane::ConstPtrs EngineImpl::GetLanes() const {
  core::Lane::ConstPtrs lanes;
  for (const auto& lane_item : data_->lanes) {
    lanes.emplace_back(lane_item.second);
  }
  return lanes;
}

core::Section::ConstPtrs EngineImpl::GetSections() const {
  core::Section::ConstPtrs sections;
  for (const auto& section_item : data_->sections) {
    sections.emplace_back(section_item.second);
  }
  return sections;
}

core::Road::ConstPtrs EngineImpl::GetRoads() const {
  core::Road::ConstPtrs roads;
  for (const auto& road_item : data_->roads) {
    roads.emplace_back(road_item.second);
  }
  return roads;
}

core::Header::ConstPtr EngineImpl::GetHeader() const { return data_->header; }

}  // namespace engine
}  // namespace opendrive
