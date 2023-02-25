#ifndef OPENDRIVE_ENGINE_CORE_LANE_H_
#define OPENDRIVE_ENGINE_CORE_LANE_H_

#include <opendrive-cpp/geometry/enums.h>

#include <memory>
#include <vector>

#include "geometry.h"
#include "id.h"

namespace opendrive {
namespace engine {
namespace core {

struct Curve {
  struct Point : public Point3D {
    double s;
    double hdg;
  };
  typedef std::vector<Point> Points;
  typedef std::vector<Point> Line;
  Line pts;
  double length = 0.;
};

struct LaneBoundaryAttr {
  double s;
  RoadMarkType boundary_type = RoadMarkType::NONE;
  RoadMarkColor boundary_color = RoadMarkColor::STANDARD;
};
typedef std::vector<LaneBoundaryAttr> LaneBoundaryAttrs;

struct LaneBoundary {
  Curve curve;
  LaneBoundaryAttrs attrs;
};

struct SpeedLimit {
  double s;
  double value;  // meters per second
};
typedef std::vector<SpeedLimit> SpeedLimits;

typedef struct Lane LaneTypedef;
struct Lane {
  typedef std::shared_ptr<LaneTypedef> Ptr;
  typedef std::shared_ptr<LaneTypedef const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  Id id;
  Curve central_curve;
  LaneBoundary left_boundary;
  LaneBoundary right_boundary;
  SpeedLimits speed_limit;
  Id parent_id;  // section id
  Ids predecessor_id;
  Ids successor_id;
  Ids left_neighbor_forward_lane_id;
  Ids right_neighbor_forward_lane_id;
  Ids left_neighbor_reverse_lane_id;
  Ids right_neighbor_reverse_lane_id;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_LANE_H_
