#include "opendrive-engine/geometry/vec2d.h"

namespace opendrive {
namespace engine {
namespace geometry {

Vec2d Vec2d::CreateUnitVec2d(double angle) {
  return Vec2d(std::cos(angle), std::sin(angle));
}

double Vec2d::Length() const { return std::hypot(x_, y_); }

double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec2d::Angle() const { return std::atan2(y_, x_); }

void Vec2d::Normalize() {
  const double l = Length();
  if (l > 1e-10) {
    x_ /= l;
    y_ /= l;
  }
}

double Vec2d::DistanceTo(const Vec2d& other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec2d::DistanceSquareTo(const Vec2d& other) const {
  return math::Square(x_ - other.x_) + math::Square(y_ - other.y_);
}

double Vec2d::CrossProd(const Vec2d& other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec2d::CrossProd(const Vec2d& end_point_1,
                        const Vec2d& end_point_2) const {
  return (end_point_1 - *this).CrossProd(end_point_2 - *this);
}

double Vec2d::CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                        const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double Vec2d::InnerProd(const Vec2d& other) const {
  return x_ * other.x() + y_ * other.y();
}
Vec2d Vec2d::Rotate(double angle) const {
  return Vec2d(x_ * cos(angle) - y_ * sin(angle),
               x_ * sin(angle) + y_ * cos(angle));
}

void Vec2d::SelfRotate(double angle) {
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2d Vec2d::operator+(const Vec2d& other) const {
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-(const Vec2d& other) const {
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(double ratio) const {
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(double ratio) const {
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d& Vec2d::operator+=(const Vec2d& other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d& Vec2d::operator-=(const Vec2d& other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d& Vec2d::operator*=(double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d& Vec2d::operator/=(double ratio) {
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d& other) const {
  return (std::abs(x_ - other.x()) < 1e-10 && std::abs(y_ - other.y()) < 1e-10);
}

// Vec2d operator*(const double ratio, const Vec2d& vec);
Vec2d operator*(const double ratio, const Vec2d& vec) { return vec * ratio; }

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive
