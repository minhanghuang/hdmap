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

class Curve {
 public:
  struct Point : public Point3D {
    double s;
    double hdg;
  };
  typedef std::vector<Point> Points;
  typedef std::vector<Point> Line;
  Line& mutable_pts() { return pts_; }
  void set_length(double d) { length_ = d; }
  const Line& pts() const { return pts_; }
  double length() const { return length_; }

 private:
  Line pts_;
  double length_ = 0;
};

class LaneBoundaryAttr {
 public:
  void set_s(double d) { s_ = d; }
  void set_boundary_type(RoadMarkType t) { boundary_type_ = t; }
  void set_boundary_color(RoadMarkColor t) { boundary_color_ = t; }
  double s() const { return s_; }
  RoadMarkType boundary_type() const { return boundary_type_; }
  RoadMarkColor boundary_color() const { return boundary_color_; }

 private:
  double s_;
  RoadMarkType boundary_type_ = RoadMarkType::NONE;
  RoadMarkColor boundary_color_ = RoadMarkColor::STANDARD;
};
typedef std::vector<LaneBoundaryAttr> LaneBoundaryAttrs;

class LaneBoundary {
 public:
  Curve& mutable_curve() { return curve_; }
  LaneBoundaryAttrs& mutable_attrs() { return attrs_; }
  const Curve& curve() const { return curve_; }
  const LaneBoundaryAttrs& attrs() const { return attrs_; }

 private:
  Curve curve_;
  LaneBoundaryAttrs attrs_;
};

class SpeedLimit {
 public:
  void set_s(double d) { s_ = d; }
  void set_value(double d) { value_ = d; }
  double s() const { return s_; }
  double value() const { return value_; }

 private:
  double s_;
  double value_;  // meters per second
};
typedef std::vector<SpeedLimit> SpeedLimits;

class Lane {
 public:
  Lane() = default;
  typedef std::shared_ptr<Lane> Ptr;
  typedef std::shared_ptr<Lane const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  void set_id(const Id& s) { id_ = s; }
  void set_parent_id(const Id& s) { parent_id_ = s; }
  void set_predecessor_ids(const Ids& s) { predecessor_ids_ = s; }
  Ids& mutable_predecessor_ids() { return predecessor_ids_; }
  void set_successor_ids(const Ids& s) { successor_ids_ = s; }
  Ids& mutable_successor_ids() { return successor_ids_; }
  Curve& mutable_central_curve() { return central_curve_; }
  LaneBoundary& mutable_left_boundary() { return left_boundary_; }
  LaneBoundary& mutable_right_boundary() { return right_boundary_; }
  SpeedLimits& mutable_speed_limits() { return speed_limits_; }
  const Id& id() const { return id_; }
  const Id& parent_id() const { return parent_id_; }
  const Ids& predecessor_ids() const { return predecessor_ids_; }
  const Ids& successor_ids() const { return successor_ids_; }
  const Ids& left_neighbor_lane_ids() const { return left_neighbor_lane_ids_; }
  const Ids& right_neighbor_lane_ids() const {
    return right_neighbor_lane_ids_;
  }
  const Curve& central_curve() const { return central_curve_; }
  const LaneBoundary& left_boundary() const { return left_boundary_; }
  const LaneBoundary& right_boundary() const { return right_boundary_; }
  const SpeedLimits& speed_limits() const { return speed_limits_; }

 private:
  Id id_;
  Id parent_id_;  // section id
  Ids predecessor_ids_;
  Ids successor_ids_;
  Ids left_neighbor_lane_ids_;
  Ids right_neighbor_lane_ids_;
  Curve central_curve_;
  LaneBoundary left_boundary_;
  LaneBoundary right_boundary_;
  SpeedLimits speed_limits_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_LANE_H_
