#include "opendrive-engine/convertor.h"

#include <memory>

namespace opendrive {
namespace engine {

common::Status Convertor::Start(const std::string& map_file,
                                core::Map::Ptr core_map) {
  core_map_ = core_map;
  if (map_file.empty() || !common::FileExists(map_file) || !core_map_) {
    return common::Status(common::ErrorCode::INIT_MAPFILE_ERROR,
                          "input file error: " + map_file);
  }
  std::unique_ptr<opendrive::Parser> xodr_perser =
      std::make_unique<opendrive::Parser>();
  opendrive::element::Map::Ptr ele_map =
      std::make_shared<opendrive::element::Map>();
  auto parse_ret = xodr_perser->ParseMap(map_file, ele_map);
  if (opendrive::ErrorCode::OK != parse_ret.error_code) {
    return common::Status(
        common::ErrorCode::CONVERTOR_XMLPARSE_ERROR,
        "map file parse error: " + map_file + " : " + parse_ret.msg);
  }
  return common::Status();
}

}  // namespace engine
}  // namespace opendrive
