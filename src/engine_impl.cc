#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

EngineImpl::EngineImpl() : param_(nullptr), data_(nullptr), kdtree_(nullptr) {}

Status EngineImpl::Init(const common::Param& param) {
  // factory load
  auto factory = cactus::Factory::Instance();
  factory->Register<common::Param>(&param, "engine_param", true);
  factory->Register<core::Data>("core_data", true);
  factory->Register<kdtree::KDTree>("kdtree", true);
  param_ = factory->GetObject<common::Param>("engine_param");
  data_ = factory->GetObject<core::Data>("core_data");
  kdtree_ = factory->GetObject<kdtree::KDTree>("kdtree");
  ENGINE_INFO("Factory Load End.");

  // convert data
  Convertor convertor;
  return convertor.Start();
}

std::string EngineImpl::GetXodrVersion() const {
  return data_->header()->rev_major() + "." + data_->header()->rev_minor() +
         "." + data_->header()->version();
}

bool EngineImpl::GetPointById(const core::Id& point_id,
                              core::Curve::Point& out_point) {
  auto split_ret = cactus::StrSplit(point_id, "_");
  core::Id lane_id = split_ret[0] + "_" + split_ret[1] + "_" + split_ret[2];
  if (0 == split_ret.size() || 0 == data_->lanes().count(lane_id)) {
    return false;
  }
  auto lane = data_->lanes().at(lane_id);
  int point_index = std::atoi(split_ret[3].c_str());
  if (!lane || point_index < 0 ||
      point_index >= lane->central_curve().pts().size()) {
    return false;
  }
  if ("1" == split_ret[4]) {
    out_point = lane->left_boundary().curve().pts().at(point_index);
  } else if ("2" == split_ret[4]) {
    out_point = lane->central_curve().pts().at(point_index);
  } else if ("3" == split_ret[4]) {
    out_point = lane->right_boundary().curve().pts().at(point_index);
  } else {
    return false;
  }
  return true;
}

core::Lane::ConstPtr EngineImpl::GetLaneById(const core::Id& id) const {
  if (data_->lanes().count(id)) {
    return data_->lanes().at(id);
  }
  return nullptr;
}

core::Section::ConstPtr EngineImpl::GetSectionById(const core::Id& id) const {
  if (data_->sections().count(id)) {
    return data_->sections().at(id);
  }
  return nullptr;
}

core::Road::ConstPtr EngineImpl::GetRoadById(const core::Id& id) const {
  if (data_->roads().count(id)) {
    return data_->roads().at(id);
  }
  return nullptr;
}

core::Lane::ConstPtrs EngineImpl::GetLanes() const {
  core::Lane::ConstPtrs lanes;
  for (const auto& lane_item : data_->lanes()) {
    lanes.emplace_back(lane_item.second);
  }
  return lanes;
}

core::Section::ConstPtrs EngineImpl::GetSections() const {
  core::Section::ConstPtrs sections;
  for (const auto& section_item : data_->sections()) {
    sections.emplace_back(section_item.second);
  }
  return sections;
}

core::Road::ConstPtrs EngineImpl::GetRoads() const {
  core::Road::ConstPtrs roads;
  for (const auto& road_item : data_->roads()) {
    roads.emplace_back(road_item.second);
  }
  return roads;
}

core::Header::ConstPtr EngineImpl::GetHeader() const { return data_->header(); }

kdtree::KDTreeResults EngineImpl::GetNearestPoints(double x, double y,
                                                   size_t num_closest) {
  return kdtree_->Query(x, y, num_closest);
}

core::Lane::ConstPtrs EngineImpl::GetNearestLanes(double x, double y,
                                                  size_t num_closest) {
  core::Lane::ConstPtrs lanes;
  auto search_ret = kdtree_->Query(x, y, num_closest);
  for (const auto& it : search_ret) {
    core::Id lane_id = common::GetLaneIdById(it.id);
    if (!lane_id.empty()) {
      lanes.emplace_back(data_->lanes().at(lane_id));
    }
  }
  return lanes;
}

}  // namespace engine
}  // namespace opendrive
