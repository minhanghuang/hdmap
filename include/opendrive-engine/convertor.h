#ifndef OPENDRIVE_ENGINE_CONVERTOR_H_
#define OPENDRIVE_ENGINE_CONVERTOR_H_

#include <memory>
#include <string>

#include "opendrive-engine/common/common.hpp"
#include "opendrive-engine/common/param.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/define.h"

namespace opendrive {
namespace engine {

class Convertor {
 public:
  Convertor() = default;
  common::Status Start(common::Param::ConstPtr param, core::Data::Ptr data);

 private:
  common::Param::ConstPtr param_;
  core::Data::Ptr data_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CONVERTOR_H_
