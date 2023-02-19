#ifndef OPENDRIVE_ENGINE_CONVERTOR_H_
#define OPENDRIVE_ENGINE_CONVERTOR_H_

#include <cactus/cactus.h>

#include <memory>
#include <string>

#include "opendrive-engine/common/common.hpp"
#include "opendrive-engine/common/log.h"
#include "opendrive-engine/common/param.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/define.h"

namespace opendrive {
namespace engine {

class Convertor {
 public:
  Convertor() = default;
  Status Start();

 private:
  inline void SetStatus(ErrorCode code, const std::string& msg);
  inline bool Next() const;
  Convertor& ConvertHeader(element::Map::Ptr ele_map);
  Convertor& ConvertRoad(element::Map::Ptr ele_map);
  Convertor& ConvertRoadAttr(const element::Road& ele_road,
                             core::Road::Ptr road);
  Convertor& ConvertJunction(element::Map::Ptr ele_map);
  Convertor& ConvertJunctionAttr(const element::Junction& ele_junction,
                                 core::Junction::Ptr junction);
  Status status_;
  common::Param::ConstPtr param_;
  core::Data::Ptr data_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CONVERTOR_H_
