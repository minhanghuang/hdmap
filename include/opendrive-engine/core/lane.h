#ifndef OPENDRIVE_ENGINE_CORE_LANE_H_
#define OPENDRIVE_ENGINE_CORE_LANE_H_

#include <cactus/macros.h>
#include <opendrive-cpp/geometry/element.h>
#include <opendrive-cpp/geometry/enums.h>

#include <memory>
#include <vector>

#include "id.h"
#include "opendrive-engine/geometry/geometry.h"

namespace opendrive {
namespace engine {
namespace core {

class Curve {
 public:
  class Point : public geometry::Point4D {
    CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, id);
    CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, start_position, 0);

   public:
    Point() : start_position_(0), id_("") {}
    Point(double x, double y)
        : geometry::Point4D(x, y), start_position_(0), id_("") {}
    Point(double x, double y, double z)
        : geometry::Point4D(x, y, z), start_position_(0), id_("") {}
    Point(double x, double y, double z, double heading)
        : geometry::Point4D(x, y, z, heading), start_position_(0), id_("") {}
    Point(double x, double y, double z, double heading, double start_position)
        : geometry::Point4D(x, y, z, heading),
          start_position_(start_position),
          id_("") {}
    Point(double x, double y, double z, double heading, double start_position,
          const Id& id)
        : geometry::Point4D(x, y, z, heading),
          start_position_(start_position),
          id_(id) {}
  };
  using Points = std::vector<Point>;
  using Line = std::vector<Point>;
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Line, pts);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, length, 0);
};

class LaneBoundaryAttr {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, start_position, 0);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(RoadMarkType, boundary_type);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(RoadMarkColor, boundary_color);

 public:
  LaneBoundaryAttr()
      : start_position_(0),
        boundary_type_(RoadMarkType::kNone),
        boundary_color_(RoadMarkColor::kStandard) {}
};
using LaneBoundaryAttrs = std::vector<LaneBoundaryAttr>;

class LaneBoundary {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Curve, curve);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(LaneBoundaryAttrs, attributes);

 public:
  LaneBoundary() {}
};

class SpeedLimit {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, start_position, 0);
  // meters per second
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, value, 0);

 public:
  SpeedLimit() {}
};
using SpeedLimits = std::vector<SpeedLimit>;

class Geomotry {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(opendrive::GeometryType, type);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Curve::Point, point);

 public:
  using Type = opendrive::GeometryType;
  Geomotry() : type_(Type::kLine) {}
};
using Geomotrys = std::vector<Geomotry>;

class Lane {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, id);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Id, parent_id);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Ids, predecessor_ids);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Ids, successor_ids);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Ids, left_neighbor_lane_ids);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Ids, right_neighbor_lane_ids);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Curve, central_curve);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(LaneBoundary, left_boundary);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(LaneBoundary, right_boundary);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(Geomotrys, geometrys);

 public:
  using Ptr = std::shared_ptr<Lane>;
  using ConstPtr = std::shared_ptr<Lane const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Lane() {}
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_LANE_H_
