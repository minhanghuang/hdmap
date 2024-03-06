#include "hdmap/engine_impl.h"

namespace hdmap {

EngineImpl::EngineImpl() : param_(nullptr), map_(nullptr), kdtree_(nullptr) {}

bool EngineImpl::Init(const Param& param) {
  param_ = std::make_shared<Param>(param);
  map_ = std::make_shared<geometry::Map>();
  kdtree_ = std::make_shared<kdtree::KDTree>();
  param_->set_step(std::max<float>(param_->step(), 0.5));

  std::shared_ptr<PipelineLoading> pipeline_data =
      std::make_shared<PipelineLoading>();
  pipeline_data->param = param_;
  pipeline_data->status = std::make_shared<Status>();
  pipeline_data->opendrive_element_map =
      std::make_shared<opendrive::element::Map>();
  pipeline_data->map = map_;
  pipeline_data->kdtree = kdtree_;

  Pipeline pipeline;
  pipeline.AddProcessor(
      std::shared_ptr<ParseProcessor>(std::make_shared<OpenDriveParser>()));
  pipeline.AddProcessor(std::shared_ptr<ConvertProcessor>(
      std::make_shared<OpenDriveConvertor>()));
  pipeline.execute(pipeline_data);

  return pipeline.ok();
}

bool EngineImpl::GetPointById(const std::string& point_id,
                              geometry::Curve::Point& out_point) {
  auto split_ret = common::Split(point_id, "_");
  std::string lane_id = split_ret[0] + "_" + split_ret[1] + "_" + split_ret[2];
  if (0 == split_ret.size() || 0 == map_->lanes().count(lane_id)) {
    return false;
  }
  auto lane = map_->lanes().at(lane_id);
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

geometry::Lane::ConstPtr EngineImpl::GetLaneById(const std::string& id) const {
  if (map_->lanes().count(id)) {
    return map_->lanes().at(id);
  }
  return nullptr;
}

geometry::Section::ConstPtr EngineImpl::GetSectionById(
    const std::string& id) const {
  if (map_->sections().count(id)) {
    return map_->sections().at(id);
  }
  return nullptr;
}

geometry::Road::ConstPtr EngineImpl::GetRoadById(const std::string& id) const {
  if (map_->roads().count(id)) {
    return map_->roads().at(id);
  }
  return nullptr;
}

geometry::Lane::ConstPtrs EngineImpl::GetLanes() const {
  geometry::Lane::ConstPtrs lanes;
  for (const auto& lane_item : map_->lanes()) {
    lanes.emplace_back(lane_item.second);
  }
  return lanes;
}

geometry::Section::ConstPtrs EngineImpl::GetSections() const {
  geometry::Section::ConstPtrs sections;
  for (const auto& section_item : map_->sections()) {
    sections.emplace_back(section_item.second);
  }
  return sections;
}

geometry::Road::ConstPtrs EngineImpl::GetRoads() const {
  geometry::Road::ConstPtrs roads;
  for (const auto& road_item : map_->roads()) {
    roads.emplace_back(road_item.second);
  }
  return roads;
}

geometry::Header::ConstPtr EngineImpl::GetHeader() const {
  return map_->header();
}

kdtree::SearchResults EngineImpl::GetNearestPoints(double x, double y,
                                                   size_t num_closest) {
  return kdtree_->Query(x, y, num_closest);
}

geometry::Lane::ConstPtrs EngineImpl::GetNearestLanes(double x, double y,
                                                      size_t num_closest) {
  geometry::Lane::ConstPtrs lanes;
  auto search_ret = kdtree_->Query(x, y, num_closest);
  for (const auto& it : search_ret) {
    std::string lane_id = common::GetLaneIdById(it.id);
    if (!lane_id.empty()) {
      lanes.emplace_back(map_->lanes().at(lane_id));
    }
  }
  return lanes;
}

}  // namespace hdmap
