#include "opendrive-engine/convertor.h"

#include <memory>
#include <string>

#include "opendrive-engine/common/log.h"

namespace opendrive {
namespace engine {

inline void Convertor::SetStatus(ErrorCode code, const std::string& msg) {
  status_.error_code = code;
  status_.msg = msg;
}

inline bool Convertor::Next() const {
  return ErrorCode::OK == status_.error_code;
}

Status Convertor::Start(common::Param::ConstPtr param, core::Data::Ptr data) {
  param_ = param;
  data_ = data;
  status_.error_code = ErrorCode::OK;
  status_.msg = "";
  std::string map_file = param_->map_file;
  if (map_file.empty() || !common::FileExists(map_file) || !data_) {
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
  if (!Next()) return *this;
  ENGINE_INFO("Convert Header Start")
  auto header = std::make_shared<core::Header>();
  header->rev_major = ele_map->header.rev_major;
  header->rev_minor = ele_map->header.rev_minor;
  header->name = ele_map->header.name;
  header->version = ele_map->header.version;
  header->date = ele_map->header.date;
  header->north = ele_map->header.north;
  header->south = ele_map->header.south;
  header->west = ele_map->header.west;
  header->east = ele_map->header.east;
  header->vendor = ele_map->header.vendor;
  data_->header = header;
  ENGINE_INFO("Convert Header End")
  return *this;
}

Convertor& Convertor::ConvertRoad(opendrive::element::Map::Ptr ele_map) {
  if (!Next()) return *this;
  ENGINE_INFO("Convert Road Start")
  for (const auto& ele_road : ele_map->roads) {
    if (ele_road.attributes.id < 0) continue;
    auto road = std::make_shared<core::Road>();
    ConvertRoadAttr(ele_road, road);
    data_->roads[road->id] = road;
  }
  ENGINE_INFO("Convert Road End")
  return *this;
}

Convertor& Convertor::ConvertRoadAttr(const element::Road& ele_road,
                                      core::Road::Ptr road) {
  if (!Next()) return *this;
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

Convertor& Convertor::ConvertJunction(opendrive::element::Map::Ptr ele_map) {
  if (!Next()) return *this;
  ENGINE_INFO("Convert Junction Start")
  for (const auto& ele_junction : ele_map->junctions) {
    if (ele_junction.attributes.id < 0) continue;
    auto junction = std::make_shared<core::Junction>();
    ConvertJunctionAttr(ele_junction, junction);
    data_->junctions[junction->id] = junction;
  }
  ENGINE_INFO("Convert Junction End")
  return *this;
}

Convertor& Convertor::ConvertJunctionAttr(const element::Junction& ele_junction,
                                          core::Junction::Ptr junction) {
  if (!Next()) return *this;
  junction->id = std::to_string(ele_junction.attributes.id);
  junction->name = ele_junction.attributes.name;
  junction->type = ele_junction.attributes.type;
  return *this;
}

}  // namespace engine
}  // namespace opendrive
