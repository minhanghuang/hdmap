#include "opendrive-engine/convertor.h"

#include <algorithm>
#include <memory>
#include <string>

#include "cactus/factory.h"
#include "opendrive-cpp/common/common.hpp"
#include "opendrive-engine/common/log.h"
#include "opendrive-engine/core/lane.h"

namespace opendrive {
namespace engine {

inline void Convertor::SetStatus(ErrorCode code, const std::string& msg) {
  status_.error_code = code;
  status_.msg = msg;
}

inline bool Convertor::Continue() const {
  return ErrorCode::OK == status_.error_code;
}

Status Convertor::Start() {
  auto factory = cactus::Factory::Instance();
  param_ = factory->GetObject<common::Param>("engine_param");
  data_ = factory->GetObject<core::Data>("core_data");
  if (!param_ || !data_) {
    SetStatus(ErrorCode::INIT_FACTORY_ERROR, "factory error.");
    return status_;
  }
  step_ = std::max<float>(0.1, param_->step);
  status_.error_code = ErrorCode::OK;
  status_.msg = "";
  std::string map_file = param_->map_file;
  if (map_file.empty() || !cactus::FileExists(map_file) || !data_) {
    SetStatus(ErrorCode::INIT_MAPFILE_ERROR, "input file error: " + map_file);
    return status_;
  }
  std::unique_ptr<opendrive::Parser> perser =
      std::make_unique<opendrive::Parser>();
  opendrive::element::Map::Ptr ele_map =
      std::make_shared<opendrive::element::Map>();
  auto parse_ret = perser->ParseMap(map_file, ele_map);
  if (opendrive::ErrorCode::OK != parse_ret.error_code) {
    SetStatus(ErrorCode::INIT_MAPFILE_ERROR, "input file error: " + map_file);
    return status_;
  }
  std::cout << "Convert Start" << std::endl;

  ConvertHeader(ele_map).ConvertRoad(ele_map).ConvertJunction(ele_map);
  return status_;
}

Convertor& Convertor::ConvertHeader(opendrive::element::Map::Ptr ele_map) {
  if (!Continue()) return *this;
  ENGINE_INFO("Convert Header Start")
  auto header = std::make_shared<core::Header>();
  header->set_rev_major(ele_map->header.rev_major);
  header->set_rev_minor(ele_map->header.rev_minor);
  header->set_name(ele_map->header.name);
  header->set_version(ele_map->header.version);
  header->set_date(ele_map->header.date);
  header->set_north(ele_map->header.north);
  header->set_south(ele_map->header.south);
  header->set_west(ele_map->header.west);
  header->set_east(ele_map->header.east);
  header->set_vendor(ele_map->header.vendor);
  data_->header = header;
  ENGINE_INFO("Convert Header End")
  return *this;
}

Convertor& Convertor::ConvertJunction(opendrive::element::Map::Ptr ele_map) {
  if (!Continue()) return *this;
  ENGINE_INFO("Convert Junction Start")
  for (const auto& ele_junction : ele_map->junctions) {
    if (ele_junction.attributes.id < 0) continue;
    auto junction = std::make_shared<core::Junction>();
    ConvertJunctionAttr(ele_junction, junction);
    data_->junctions[junction->id()] = junction;
  }
  ENGINE_INFO("Convert Junction End")
  return *this;
}

Convertor& Convertor::ConvertJunctionAttr(const element::Junction& ele_junction,
                                          core::Junction::Ptr junction) {
  if (!Continue()) return *this;
  junction->set_id(std::to_string(ele_junction.attributes.id));
  junction->set_name(ele_junction.attributes.name);
  junction->set_type(ele_junction.attributes.type);
  return *this;
}

Convertor& Convertor::ConvertRoad(opendrive::element::Map::Ptr ele_map) {
  if (!Continue()) return *this;
  ENGINE_INFO("Convert Road Start")
  for (const auto& ele_road : ele_map->roads) {
    if (ele_road.attributes.id < 0) continue;
    auto road = std::make_shared<core::Road>();
    ConvertRoadAttr(ele_road, road).ConvertSection(ele_road, road);
    data_->roads[road->id] = road;
  }
  ENGINE_INFO("Convert Road End")
  return *this;
}

Convertor& Convertor::ConvertRoadAttr(const element::Road& ele_road,
                                      core::Road::Ptr road) {
  if (!Continue()) return *this;
  road->id = std::to_string(ele_road.attributes.id);
  road->name = ele_road.attributes.name;
  road->junction_id = -1 == ele_road.attributes.junction
                          ? ""
                          : std::to_string(ele_road.attributes.junction);
  road->length = ele_road.attributes.length;
  road->rule = ele_road.attributes.rule;
  if (-1 != ele_road.link.predecessor.id) {
    // road predecessor/successor range 0~1 in opendrive.
    road->predecessor_id.emplace(std::to_string(ele_road.link.predecessor.id));
  }
  if (-1 != ele_road.link.successor.id) {
    road->successor_id.emplace(std::to_string(ele_road.link.successor.id));
  }
  for (const auto& ele_info : ele_road.type_info) {
    core::RoadInfo road_info;
    road_info.s = ele_info.s;
    road_info.type = ele_info.type;
    road->info.emplace_back(road_info);
  }
  return *this;
}

Convertor& Convertor::ConvertSection(const element::Road& ele_road,
                                     core::Road::Ptr road) {
  if (!Continue()) return *this;

  std::string road_id = std::to_string(ele_road.attributes.id);
  double road_ds = 0;
  int section_idx = 0;
  for (const auto& ele_section : ele_road.lanes.lane_sections) {
    auto section = std::make_shared<core::Section>();
    road->sections.emplace_back(section);
    section->set_id(road_id + "_" + std::to_string(section_idx++));
    section->set_parent_id(road_id);
    section->set_start_position(ele_section.s0);
    section->set_end_position(ele_section.s1);
    section->set_length(ele_section.s1 - ele_section.s0);
    data_->sections[section->id()] = section;

    /// center lane
    if (1 != ele_section.center.lanes.size()) {
      SetStatus(ErrorCode::CONVERTOR_CENTERLANE_ERROR,
                section->id() + " center lane size not equal 1.");
      return *this;
    } else {
      auto lane = std::make_shared<core::Lane>();
      section->mutable_center_lane() = lane;
      lane->set_id(section->id() + "_0");
      lane->set_parent_id(section->id());
      CenterLaneSampling(ele_road.plan_view.geometrys,
                         ele_road.lanes.lane_offsets, section, road_ds);
      data_->lanes[lane->id()] = lane;
    }
    // 参考线: 中心车道的左边界
    core::Curve::Line& refe_line = section->mutable_center_lane()
                                       ->mutable_left_boundary()
                                       .mutable_curve()
                                       .mutable_pts();

    /// left lanes
    for (const auto& ele_lane : ele_section.left.lanes) {
      auto lane = std::make_shared<core::Lane>();
      section->mutable_left_lanes().emplace_back(lane);
      lane->set_id(section->id() + "_" +
                   std::to_string(ele_lane.attributes.id));
      lane->set_parent_id(section->id());
      LaneSampling(ele_lane, lane, refe_line);
      data_->lanes[lane->id()] = lane;
      refe_line = lane->mutable_right_boundary().mutable_curve().mutable_pts();
    }
    // 参考线: 中心车道的右边界
    refe_line = section->mutable_center_lane()
                    ->mutable_right_boundary()
                    .mutable_curve()
                    .mutable_pts();

    /// right lanes
    for (const auto& ele_lane : ele_section.right.lanes) {
      auto lane = std::make_shared<core::Lane>();
      section->mutable_right_lanes().emplace_back(lane);
      lane->set_id(section->id() + "_" +
                   std::to_string(ele_lane.attributes.id));
      lane->set_parent_id(section->id());
      LaneSampling(ele_lane, lane, refe_line);
      data_->lanes[lane->id()] = lane;
      refe_line = lane->mutable_right_boundary().mutable_curve().mutable_pts();
    }
  }

  return *this;
}

void Convertor::CenterLaneSampling(const element::Geometry::Ptrs& geometrys,
                                   const element::LaneOffsets& lane_offsets,
                                   core::Section::Ptr section,
                                   double& road_ds) {
  double section_ds = 0;
  element::Geometry::Ptr geometry = nullptr;
  core::Curve::Point point;
  element::Point refe_point;
  element::Point offset_point;
  section->mutable_center_lane()->mutable_central_curve().mutable_pts().clear();

  while (true) {
    if (section_ds >= section->length()) {
      break;
    }
    geometry = GetGeometry(geometrys, road_ds);
    if (!geometry) {
      break;
    }
    refe_point = geometry->GetPoint(road_ds);
    double offset = GetLaneOffsetValue(lane_offsets, road_ds);
    if (0 != offset) {
      offset_point = opendrive::common::GetOffsetPoint(refe_point, offset);
      point.x = offset_point.x;
      point.y = offset_point.y;
      point.hdg = offset_point.hdg;
      point.s = section_ds;
    } else {
      point.x = refe_point.x;
      point.y = refe_point.y;
      point.hdg = refe_point.hdg;
      point.s = section_ds;
    }
    section->mutable_center_lane()
        ->mutable_central_curve()
        .mutable_pts()
        .emplace_back(point);
    section->mutable_center_lane()
        ->mutable_left_boundary()
        .mutable_curve()
        .mutable_pts()
        .emplace_back(point);
    section->mutable_center_lane()
        ->mutable_right_boundary()
        .mutable_curve()
        .mutable_pts()
        .emplace_back(point);

    section_ds += step_;
    road_ds += step_;
  }
}

void Convertor::LaneSampling(const element::Lane& ele_lane,
                             core::Lane::Ptr lane,
                             const core::Curve::Line& refe_line) {
  core::Curve::Point right_point;
  core::Curve::Point center_point;
  auto lane_idx = opendrive::common::Split(lane->id(), "_");
  const int lane_dir = lane_idx.at(2) > "0" ? 1 : -1;
  for (const auto& refe_point : refe_line) {
    double lane_width = ele_lane.GetLaneWidth(refe_point.s) * lane_dir;
    center_point =
        opendrive::common::GetOffsetPoint(refe_point, lane_width / 2.0);
    right_point = opendrive::common::GetOffsetPoint(refe_point, lane_width);
    lane->mutable_left_boundary().mutable_curve().mutable_pts().emplace_back(
        refe_point);
    lane->mutable_central_curve().mutable_pts().emplace_back(center_point);
    lane->mutable_right_boundary().mutable_curve().mutable_pts().emplace_back(
        right_point);
  }
}

element::Geometry::Ptr Convertor::GetGeometry(
    const element::Geometry::Ptrs& geometrys, double road_ds) {
  auto geometry_idx = opendrive::common::GetGtPtrPoloy3(geometrys, road_ds);
  if (geometry_idx < 0) {
    SetStatus(ErrorCode::CONVERTOR_CENTERLANE_ERROR,
              "get geometry index execption.");
    return nullptr;
  }
  if (geometry_idx && geometry_idx < geometrys.size()) {
    return geometrys.at(geometry_idx);
  }
  return nullptr;
}

double Convertor::GetLaneOffsetValue(const element::LaneOffsets& offsets,
                                     double road_ds) {
  int offset_idx = opendrive::common::GetGeValuePoloy3(offsets, road_ds);
  if (offset_idx >= 0 && offset_idx < offsets.size()) {
    return offsets.at(offset_idx).GetOffsetValue(road_ds);
  }
  return 0;
}

}  // namespace engine
}  // namespace opendrive
