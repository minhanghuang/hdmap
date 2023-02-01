#include "opendrive-engine/convertor.h"

#include <memory>

namespace opendrive {
namespace engine {

common::Status Convertor::Start(common::Param::ConstPtr param,
                                core::Data::Ptr data) {
  param_ = param;
  data_ = data;
  std::string map_file = param_->map_file;
  if (map_file.empty() || !common::FileExists(map_file) || !data_) {
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
