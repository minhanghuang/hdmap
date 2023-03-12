#include "opendrive-engine/geometry/line_segment2d.h"

namespace opendrive {
namespace engine {
namespace geometry {

bool IsWithin(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - 1e-10 && val <= bound2 + 1e-10;
}

LineSegment2d::LineSegment2d() : heading_(0), length_(0) {
  unit_direction_ = geometry::Vec2d(1, 0);
}

LineSegment2d::LineSegment2d(const geometry::Vec2d& start,
                             const geometry::Vec2d& end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= 1e-10 ? geometry::Vec2d(0, 0)
                        : geometry::Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();
}

geometry::Vec2d LineSegment2d::rotate(const double angle) {
  geometry::Vec2d diff_vec = end_ - start_;
  diff_vec.SelfRotate(angle);
  return start_ + diff_vec;
}

double LineSegment2d::length() const { return length_; }

double LineSegment2d::length_sqr() const { return length_ * length_; }

double LineSegment2d::DistanceTo(const geometry::Vec2d& point) const {
  if (length_ <= 1e-10) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceTo(const geometry::Vec2d& point,
                                 geometry::Vec2d* const nearest_pt) const {
  if (length_ <= 1e-10) {
    *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const geometry::Vec2d& point) const {
  if (length_ <= 1e-10) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return math::Square(x0) + math::Square(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return math::Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(
    const geometry::Vec2d& point, geometry::Vec2d* const nearest_pt) const {
  if (length_ <= 1e-10) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return math::Square(x0) + math::Square(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return math::Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool LineSegment2d::IsPointIn(const geometry::Vec2d& point) const {
  if (length_ <= 1e-10) {
    return std::abs(point.x() - start_.x()) <= 1e-10 &&
           std::abs(point.y() - start_.y()) <= 1e-10;
  }
  // const double prod = CrossProd(point, start_, end_);
  const double prod = point.CrossProd(start_, end_);
  if (std::abs(prod) > 1e-10) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

double LineSegment2d::ProjectOntoUnit(const geometry::Vec2d& point) const {
  return unit_direction_.InnerProd(point - start_);
}

double LineSegment2d::ProductOntoUnit(const geometry::Vec2d& point) const {
  return unit_direction_.CrossProd(point - start_);
}

bool LineSegment2d::HasIntersect(const LineSegment2d& other_segment) const {
  geometry::Vec2d point;
  return GetIntersect(other_segment, &point);
}

bool LineSegment2d::GetIntersect(const LineSegment2d& other_segment,
                                 geometry::Vec2d* const point) const {
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= 1e-10 || other_segment.length() <= 1e-10) {
    return false;
  }
  // const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc1 = start_.CrossProd(end_, other_segment.start());
  // const double cc2 = CrossProd(start_, end_, other_segment.end());
  const double cc2 = start_.CrossProd(end_, other_segment.end());
  if (cc1 * cc2 >= -1e-10) {
    return false;
  }
  // const double cc3 = CrossProd(other_segment.start(), other_segment.end(),
  // start_);
  const double cc3 =
      other_segment.start().CrossProd(other_segment.end(), start_);
  // const double cc4 = CrossProd(other_segment.start(), other_segment.end(),
  // end_);
  const double cc4 = other_segment.start().CrossProd(other_segment.end(), end_);
  if (cc3 * cc4 >= -1e-10) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = geometry::Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                           start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double LineSegment2d::GetPerpendicularFoot(
    const geometry::Vec2d& point, geometry::Vec2d* const foot_point) const {
  if (length_ <= 1e-10) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive
