#ifndef OPENDRIVE_ENGINE_CONVERTOR_H_
#define OPENDRIVE_ENGINE_CONVERTOR_H_

#include <memory>
#include <string>

#include "opendrive-engine/common/common.hpp"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/map.h"

namespace opendrive {
namespace engine {

class Convertor {
 public:
  Convertor() = default;
  common::Status Start(const std::string& map_file, core::Map::Ptr core_map);

 private:
  core::Map::Ptr core_map_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CONVERTOR_H_
