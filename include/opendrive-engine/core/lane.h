#ifndef OPENDRIVE_ENGINE_CORE_LANE_H_
#define OPENDRIVE_ENGINE_CORE_LANE_H_

#include <opendrive-cpp/geometry/element.h>
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
  class Point : public element::Point {
   public:
    Point() : start_position_(0) {}
    Point(double x, double y) : element::Point(x, y), start_position_(0) {}
    Point(double x, double y, double z)
        : element::Point(x, y, z), start_position_(0) {}
    Point(double x, double y, double z, double heading)
        : element::Point(x, y, z, heading), start_position_(0) {}
    Point(double x, double y, double z, double heading, double start_position)
        : element::Point(x, y, z, heading), start_position_(start_position) {}
    void set_start_position(double d) { start_position_ = d; }
    double& mutable_start_position() { return start_position_; }
    double start_position() const { return start_position_; }

   private:
    double start_position_;
  };
  typedef std::vector<Point> Points;
  typedef std::vector<Point> Line;
  void set_pts(const Line& v) { pts_ = v; }
  void set_length(double d) { length_ = d; }
  Line& mutable_pts() { return pts_; }
  double& mutable_length() { return length_; }
  const Line& pts() const { return pts_; }
  double length() const { return length_; }

 private:
  Line pts_;
  double length_ = 0;
};

class LaneBoundaryAttr {
 public:
  LaneBoundaryAttr()
      : start_position_(0),
        boundary_type_(RoadMarkType::NONE),
        boundary_color_(RoadMarkColor::STANDARD) {}
  void set_s(double d) { start_position_ = d; }
  void set_boundary_type(RoadMarkType t) { boundary_type_ = t; }
  void set_boundary_color(RoadMarkColor t) { boundary_color_ = t; }
  double& mutable_start_position() { return start_position_; }
  RoadMarkType& mutable_boundary_type() { return boundary_type_; }
  RoadMarkColor& mutable_boundary_color() { return boundary_color_; }
  double s() const { return start_position_; }
  RoadMarkType boundary_type() const { return boundary_type_; }
  RoadMarkColor boundary_color() const { return boundary_color_; }

 private:
  double start_position_;
  RoadMarkType boundary_type_;
  RoadMarkColor boundary_color_;
};
typedef std::vector<LaneBoundaryAttr> LaneBoundaryAttrs;

class LaneBoundary {
 public:
  void set_curve(const Curve& v) { curve_ = v; }
  void set_attrs(const LaneBoundaryAttrs& v) { attrs_ = v; }
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
  SpeedLimit() : start_position_(0), value_(0) {}
  void set_s(double d) { start_position_ = d; }
  void set_value(double d) { value_ = d; }
  double& mutable_start_position() { return start_position_; }
  double& mutable_value() { return value_; }
  double start_position() const { return start_position_; }
  double value() const { return value_; }

 private:
  double start_position_;
  double value_;  // meters per second
};
typedef std::vector<SpeedLimit> SpeedLimits;

class Lane {
 public:
  typedef std::shared_ptr<Lane> Ptr;
  typedef std::shared_ptr<Lane const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  Lane() : id_(""), parent_id_("") {}
  void set_id(const Id& s) { id_ = s; }
  void set_parent_id(const Id& s) { parent_id_ = s; }
  void set_predecessor_ids(const Ids& v) { predecessor_ids_ = v; }
  void set_successor_ids(const Ids& v) { successor_ids_ = v; }
  void set_left_neighbor_lane_ids(const Ids& v) { left_neighbor_lane_ids_ = v; }
  void set_right_neighbor_lane_ids(const Ids& v) {
    right_neighbor_lane_ids_ = v;
  }
  void set_central_curve(const Curve& v) { central_curve_ = v; }
  void set_left_boundary(const LaneBoundary& v) { left_boundary_ = v; }
  void set_right_boundary(const LaneBoundary& v) { right_boundary_ = v; }
  void set_speed_limits(const SpeedLimits& v) { speed_limits_ = v; }
  Id& mutable_id() { return id_; }
  Id& mutable_parent_id() { return parent_id_; }
  Ids& mutable_predecessor_ids() { return predecessor_ids_; }
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
