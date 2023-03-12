#ifndef OPENDRIVE_ENGINE_MATH_H_
#define OPENDRIVE_ENGINE_MATH_H_

#include <cmath>
#include <utility>

namespace opendrive {
namespace engine {
namespace math {

template <typename T>
T Square(const T value) {
  return value * value;
}

template <typename T>
double EuclideanDistance(const T& p0, const T& p1) {
  return std::sqrt(std::pow(p0.x() - p1.x(), 2.0) +
                   std::pow(p0.y() - p1.y(), 2.0));
}

template <typename T>
double EuclideanDistance(T x0, T y0, T x1, T y1) {
  return std::sqrt(std::pow(x0 - x1, 2.0) + std::pow(y0 - y1, 2.0));
}

template <typename T>
double ManhattanDistance(const T& p0, const T& p1) {
  return std::abs(p0.x() - p1.x()) + std::abs(p0.y() - p1.y());
}

template <typename T>
double ManhattanDistance(T x0, T y0, T x1, T y1) {
  return std::abs(x0 - x1) + std::abs(y0 - y1);
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

double NormalizeAngle(double angle);

double CrossProd(double x0, double y0, double x1, double y1);

double InnerProd(double x0, double y0, double x1, double y1);

double WrapAngle(double angle);

double AngleDiff(double from, double to);

int RandomInt(int s, int t, unsigned int rand_seed);

double RandomDouble(double s, double t, unsigned int rand_seed);

double Gaussian(double u, double stand, double x);

std::pair<double, double> Cartesian2Polar(double x, double y);

}  // namespace math
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_MATH_H_
