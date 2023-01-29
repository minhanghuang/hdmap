#ifndef OPENDRIVE_ENGINE_CORE_GEOMETRY_H_
#define OPENDRIVE_ENGINE_CORE_GEOMETRY_H_

#include <string>

#include "opendrive-cpp/opendrive.h"

namespace opendrive {
namespace engine {
namespace core {

struct Point2D {
  double x = 0.;
  double y = 0.;
};

struct Point3D {
  double x = 0.;
  double y = 0.;
  double z = 0.;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_GEOMETRY_H_
