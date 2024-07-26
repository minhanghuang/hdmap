#include "hdmap_engine/adapter/opendrive/opendrive_convertor.h"

namespace hdmap {

bool OpenDriveConvertor::process(std::shared_ptr<PipelineData> pipeline_data) {
  return CheckIn(pipeline_data).Convert().BuildKDTree().Finish();
}

OpenDriveConvertor& OpenDriveConvertor::CheckIn(
    std::shared_ptr<PipelineData> pipeline_data) {
  if (nullptr == pipeline_data) {
    UpdateStatus(ErrorCode::kInitError, "");
    return *this;
  }
  pipeline_data_ = std::static_pointer_cast<PipelineLoading>(pipeline_data);
  if (nullptr == pipeline_data_) {
    UpdateStatus(ErrorCode::XML_ROAD_ELEMENT_ERROR, "");
    return *this;
  }
  if (nullptr == pipeline_data_->opendrive_element_map ||
      nullptr == pipeline_data_->param || nullptr == pipeline_data_->status ||
      nullptr == pipeline_data_->map || nullptr == pipeline_data_->kdtree) {
    UpdateStatus(ErrorCode::kInitError, "");
    return *this;
  }
  return *this;
}

OpenDriveConvertor& OpenDriveConvertor::Convert() {
  ConvertHeader(pipeline_data_->opendrive_element_map)
      .ConvertRoad(pipeline_data_->opendrive_element_map);
  return *this;
}

bool OpenDriveConvertor::Finish() {
  if (!Continue()) return false;
  return Continue();
}

OpenDriveConvertor& OpenDriveConvertor::ConvertHeader(
    opendrive::element::Map::Ptr ele_map) {
  if (!Continue()) return *this;
  auto header = std::make_shared<geometry::Header>();
  header->set_rev_major(ele_map->header().rev_major());
  header->set_rev_minor(ele_map->header().rev_minor());
  header->set_name(ele_map->header().name());
  header->set_version(ele_map->header().version());
  header->set_date(ele_map->header().date());
  header->set_north(ele_map->header().north());
  header->set_south(ele_map->header().south());
  header->set_west(ele_map->header().west());
  header->set_east(ele_map->header().east());
  header->set_vendor(ele_map->header().vendor());
  pipeline_data_->map->set_header(header);
  return *this;
}

OpenDriveConvertor& OpenDriveConvertor::ConvertRoad(
    opendrive::element::Map::Ptr ele_map) {
  if (!Continue()) return *this;
  for (const auto& ele_road : ele_map->roads()) {
    if (ele_road.attribute().id() < 0) continue;
    auto road = std::make_shared<geometry::Road>();
    ConvertRoadAttr(ele_road, road);
    ConvertSections(ele_road, road);
    pipeline_data_->map->mutable_roads()->emplace(
        std::make_pair(road->id(), road));
  }
  return *this;
}

OpenDriveConvertor& OpenDriveConvertor::ConvertRoadAttr(
    const opendrive::element::Road& ele_road, geometry::Road::Ptr road) {
  if (!Continue()) return *this;
  road->set_id(std::to_string(ele_road.attribute().id()));
  road->set_name(ele_road.attribute().name());
  road->set_length(ele_road.attribute().length());
  if (-1 != ele_road.link().predecessor().id()) {
    // road predecessor/successor range 0~1 in opendrive.
    road->mutable_predecessor_ids()->emplace_back(
        std::to_string(ele_road.link().predecessor().id()));
  }
  if (-1 != ele_road.link().successor().id()) {
    road->mutable_successor_ids()->emplace_back(
        std::to_string(ele_road.link().successor().id()));
  }
  return *this;
}

OpenDriveConvertor& OpenDriveConvertor::ConvertSections(
    const opendrive::element::Road& ele_road, geometry::Road::Ptr road) {
  if (!Continue()) return *this;

  std::string road_id = std::to_string(ele_road.attribute().id());
  double road_ds = 0;
  int section_idx = 0;
  for (const auto& ele_section : ele_road.lanes().lane_sections()) {
    auto section = std::make_shared<geometry::Section>();
    road->mutable_sections()->emplace_back(section);
    section->set_id(road_id + "_" + std::to_string(section_idx++));
    section->set_parent_id(road_id);
    section->set_start_position(ele_section.start_position());
    section->set_end_position(ele_section.end_position());
    section->set_length(ele_section.end_position() -
                        ele_section.start_position());
    pipeline_data_->map->mutable_sections()->emplace(
        std::make_pair(section->id(), section));

    /// center lane
    if (1 != ele_section.center().lanes().size()) {
      UpdateStatus(ErrorCode::kConvertorCenterlaneError,
                   section->id() + " center lane size not equal 1.");
      return *this;
    } else {
      auto lane = std::make_shared<geometry::Lane>();
      section->set_center_lane(lane);
      // lane attr
      lane->set_id(section->id() + "_0");
      lane->set_parent_id(section->id());
      CenterLaneSampling(ele_road.plan_view().geometrys(),
                         ele_road.lanes().lane_offsets(), section, road_ds);
      pipeline_data_->map->mutable_lanes()->emplace(
          std::make_pair(lane->id(), lane));
    }
    // 参考线: 中心车道的左边界
    geometry::Curve::Line* refe_line = section->mutable_center_lane()
                                           ->mutable_left_boundary()
                                           ->mutable_curve()
                                           ->mutable_pts();

    /// left lanes
    for (const auto& ele_lane : ele_section.left().lanes()) {
      auto lane = std::make_shared<geometry::Lane>();
      section->mutable_left_lanes()->emplace_back(lane);
      lane->set_id(section->id() + "_" +
                   std::to_string(ele_lane.attribute().id()));
      lane->set_parent_id(section->id());
      LaneSampling(ele_lane, lane, refe_line);
      pipeline_data_->map->mutable_lanes()->emplace(
          std::make_pair(lane->id(), lane));
      refe_line =
          lane->mutable_right_boundary()->mutable_curve()->mutable_pts();
    }
    // 参考线: 中心车道的右边界
    refe_line = section->mutable_center_lane()
                    ->mutable_right_boundary()
                    ->mutable_curve()
                    ->mutable_pts();

    /// right lanes
    for (const auto& ele_lane : ele_section.right().lanes()) {
      auto lane = std::make_shared<geometry::Lane>();
      section->mutable_right_lanes()->emplace_back(lane);
      lane->set_id(section->id() + "_" +
                   std::to_string(ele_lane.attribute().id()));
      lane->set_parent_id(section->id());
      LaneSampling(ele_lane, lane, refe_line);
      pipeline_data_->map->mutable_lanes()->emplace(
          std::make_pair(lane->id(), lane));
      refe_line =
          lane->mutable_right_boundary()->mutable_curve()->mutable_pts();
    }
  }

  return *this;
}

OpenDriveConvertor& OpenDriveConvertor::BuildKDTree() {
  if (!Continue()) return *this;
  pipeline_data_->kdtree->Init(center_line_pts_);
  return *this;
}

void OpenDriveConvertor::AppendKDTreeSample(
    const geometry::Curve::Point& point) {
  center_line_pts_.emplace_back(point);
}

void OpenDriveConvertor::CenterLaneSampling(
    const opendrive::element::Geometry::Ptrs& geometrys,
    const opendrive::element::LaneOffsets& lane_offsets,
    geometry::Section::Ptr section, double& road_ds) {
  double section_ds = 0;
  geometry::Curve::Point point;
  opendrive::element::Point refe_point;
  opendrive::element::Point offset_point;
  opendrive::element::Geometry::ConstPtr geometry = nullptr;
  int geometry_type = -1;
  size_t point_idx = 0;
  bool last_point = false;
  section->mutable_center_lane()
      ->mutable_central_curve()
      ->mutable_pts()
      ->clear();

  while (true) {
    geometry = GetGeometry(geometrys, road_ds);
    if (!geometry) {
      break;
    }
    refe_point = geometry->GetPoint(road_ds);
    double offset = GetLaneOffsetValue(lane_offsets, road_ds);
    if (0 != offset) {
      offset_point =
          common::GetOffsetPoint<opendrive::element::Point>(refe_point, offset);
      point.set_x(offset_point.x());
      point.set_y(offset_point.y());
      point.set_heading(offset_point.heading());
      point.set_start_position(section_ds);
      point.set_id(section->center_lane()->id() + "_" +
                   std::to_string(point_idx++));
    } else {
      point.set_x(refe_point.x());
      point.set_y(refe_point.y());
      point.set_heading(refe_point.heading());
      point.set_start_position(section_ds);
      point.set_id(section->center_lane()->id() + "_" +
                   std::to_string(point_idx++));
    }
    if (geometry_type != static_cast<int>(geometry->type())) {
      // new geometry
      geometry::Curve::Point geometry_point(point);
      geometry_point.set_id(geometry_point.id() + "_2");
      geometry_type = static_cast<int>(geometry->type());
    }
    {
      /// 参考线车道 左边界
      geometry::Curve::Point reference_point(point);
      reference_point.set_id(reference_point.id() + "_1");
      section->mutable_center_lane()
          ->mutable_left_boundary()
          ->mutable_curve()
          ->mutable_pts()
          ->emplace_back(reference_point);
    }
    {
      /// 参考线车道 中线
      geometry::Curve::Point reference_point(point);
      reference_point.set_id(reference_point.id() + "_2");
      section->mutable_center_lane()
          ->mutable_central_curve()
          ->mutable_pts()
          ->emplace_back(reference_point);
    }
    {
      /// 参考线车道 右边界
      geometry::Curve::Point reference_point(point);
      reference_point.set_id(reference_point.id() + "_3");
      section->mutable_center_lane()
          ->mutable_right_boundary()
          ->mutable_curve()
          ->mutable_pts()
          ->emplace_back(reference_point);
    }

    if (last_point) {
      break;
    } else if ((section_ds + pipeline_data_->param->step() >=
                section->length())) {
      //  last point
      road_ds -= (section_ds - section->length());
      section_ds = section->length();
      last_point = true;
    } else {
      section_ds += pipeline_data_->param->step();
      road_ds += pipeline_data_->param->step();
    }
  }
}

void OpenDriveConvertor::LaneSampling(const opendrive::element::Lane& ele_lane,
                                      geometry::Lane::Ptr lane,
                                      const geometry::Curve::Line* refe_line) {
  geometry::Curve::Point point;
  int point_idx = 0;
  auto lane_idx = common::Split(lane->id(), "_");
  const int lane_dir = lane_idx.at(2) > "0" ? 1 : -1;
  double lane_width = 0;
  std::string point_id = "";
  for (const auto& refe_point : *refe_line) {
    lane_width = ele_lane.GetLaneWidth(refe_point.start_position()) * lane_dir;
    std::cout << refe_point.start_position() << ": " << lane_width << std::endl;
    point_id = lane->id() + "_" + std::to_string(point_idx++);

    // left boundary point
    point = refe_point;
    point.set_id(point_id + "_1");
    lane->mutable_left_boundary()->mutable_curve()->mutable_pts()->emplace_back(
        point);

    // center line point
    point = common::GetOffsetPoint<geometry::Curve::Point>(refe_point,
                                                           lane_width / 2.0);
    point.set_id(point_id + "_2");
    lane->mutable_central_curve()->mutable_pts()->emplace_back(point);
    AppendKDTreeSample(point);

    // right boundary point
    point =
        common::GetOffsetPoint<geometry::Curve::Point>(refe_point, lane_width);
    point.set_id(point_id + "_3");
    lane->mutable_right_boundary()
        ->mutable_curve()
        ->mutable_pts()
        ->emplace_back(point);
  }
}

opendrive::element::Geometry::ConstPtr OpenDriveConvertor::GetGeometry(
    const opendrive::element::Geometry::Ptrs& geometrys, double road_ds) {
  auto geometry_idx = common::GetGtPtrPoloy3(geometrys, road_ds);
  if (geometry_idx < 0) {
    UpdateStatus(ErrorCode::kConvertorCenterlaneError,
                 "get geometry index execption.");
    return nullptr;
  }
  if (geometry_idx < geometrys.size()) {
    return geometrys.at(geometry_idx);
  }
  return nullptr;
}

double OpenDriveConvertor::GetLaneOffsetValue(
    const opendrive::element::LaneOffsets& offsets, double road_ds) {
  int offset_idx = common::GetGeValuePoloy3(offsets, road_ds);
  if (offset_idx >= 0 && offset_idx < offsets.size()) {
    return offsets.at(offset_idx).GetOffsetValue(road_ds);
  }
  return 0;
}

}  // namespace hdmap
