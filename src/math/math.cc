#include "opendrive-engine/math/math.h"

namespace opendrive {
namespace engine {
namespace math {

double NormalizeAngle(double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double CrossProd(double x0, double y0, double x1, double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(double x0, double y0, double x1, double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double AngleDiff(double from, double to) { return NormalizeAngle(to - from); }

double Gaussian(double u, double stand, double x) {
  return (1.0 / std::sqrt(2 * M_PI * stand * stand)) *
         std::exp(-(x - u) * (x - u) / (2 * stand * stand));
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  const double r = std::sqrt(x * x + y * y);
  const double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

}  // namespace math
}  // namespace engine
}  // namespace opendrive
