#include "opendrive-engine/geometry/euler_angles_zxy.h"

namespace opendrive {
namespace engine {
namespace geometry {

EulerAnglesZXY::EulerAnglesZXY() : roll_(0), pitch_(0), yaw_(0) {}

EulerAnglesZXY::EulerAnglesZXY(double yaw) : roll_(0), pitch_(0), yaw_(yaw) {}

EulerAnglesZXY::EulerAnglesZXY(double roll, double pitch, double yaw)
    : roll_(roll), pitch_(pitch), yaw_(yaw) {}

EulerAnglesZXY::EulerAnglesZXY(double qw, double qx, double qy, double qz)
    : roll_(std::atan2(
          2.0 * (qw * qy - qx * qz),
          2.0 * (math::Square<double>(qw) + math::Square<double>(qz)) - 1.0)),
      pitch_(std::asin(2.0 * (qw * qx + qy * qz))),
      yaw_(std::atan2(
          2.0 * (qw * qz - qx * qy),
          2.0 * (math::Square<double>(qw) + math::Square<double>(qy)) - 1.0)) {}

EulerAnglesZXY::EulerAnglesZXY(const geometry::Quaternion& q)
    : EulerAnglesZXY(q.w(), q.x(), q.y(), q.z()) {}

void EulerAnglesZXY::Normalize() {
  roll_ = math::NormalizeAngle(roll_);
  pitch_ = math::NormalizeAngle(pitch_);
  yaw_ = math::NormalizeAngle(yaw_);
}

bool EulerAnglesZXY::IsValid() {
  Normalize();
  return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
}

geometry::Quaternion EulerAnglesZXY::ToQuaternion() const {
  const double coeff = 0.5;
  double r = roll_ * coeff;
  double p = pitch_ * coeff;
  double y = yaw_ * coeff;

  double sr = std::sin(r);
  double sp = std::sin(p);
  double sy = std::sin(y);

  double cr = std::cos(r);
  double cp = std::cos(p);
  double cy = std::cos(y);

  double qw = cr * cp * cy - sr * sp * sy;
  double qx = cr * sp * cy - sr * cp * sy;
  double qy = cr * sp * sy + sr * cp * cy;
  double qz = cr * cp * sy + sr * sp * cy;
  if (qw < 0.0) {
    return {-qw, -qx, -qy, -qz};
  }
  return {qw, qx, qy, qz};
}

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive
