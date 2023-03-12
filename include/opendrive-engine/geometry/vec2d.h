#ifndef OPENDRIVE_ENGINE_GEOMETRY_VEC2D_H_
#define OPENDRIVE_ENGINE_GEOMETRY_VEC2D_H_

#include "opendrive-engine/math/math.h"

namespace opendrive {
namespace engine {
namespace geometry {

class Vec2d {
 public:
  Vec2d() : x_(0), y_(0) {}
  Vec2d(double x, double y) : x_(x), y_(y) {}
  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  double& mutable_x() { return x_; }
  double& mutable_y() { return y_; }
  double x() const { return x_; }
  double y() const { return y_; }

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2d CreateUnitVec2d(double angle);

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2d& other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec2d& other) const;

  //! Returns the "cross" product between these two Vec2d (non-standard).
  double CrossProd(const Vec2d& other) const;

  /**
   * @brief Cross product between two 2-D vectors from the common start point,
   *        and end at two other points.
   * @param end_point_1 The end point of the first vector.
   * @param end_point_2 The end point of the second vector.
   *
   * @return The cross product result.
   */
  double CrossProd(const Vec2d& end_point_1, const Vec2d& end_point_2) const;

  /**
   * @brief Cross product between two 2-D vectors from the common start point,
   *        and end at two other points.
   * @param start_point The common start point of two vectors in 2-D.
   * @param end_point_1 The end point of the first vector.
   * @param end_point_2 The end point of the second vector.
   *
   * @return The cross product result.
   */
  static double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                          const Vec2d& end_point_2);

  //! Returns the inner product between these two Vec2d.
  double InnerProd(const Vec2d& other) const;

  //! rotate the vector by angle.
  Vec2d Rotate(double angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(double angle);

  //! Sums two Vec2d
  Vec2d operator+(const Vec2d& other) const;

  //! Subtracts two Vec2d
  Vec2d operator-(const Vec2d& other) const;

  //! Multiplies Vec2d by a scalar
  Vec2d operator*(double ratio) const;

  //! Divides Vec2d by a scalar
  Vec2d operator/(double ratio) const;

  //! Sums another Vec2d to the current one
  Vec2d& operator+=(const Vec2d& other);

  //! Subtracts another Vec2d to the current one
  Vec2d& operator-=(const Vec2d& other);

  //! Multiplies this Vec2d by a scalar
  Vec2d& operator*=(double ratio);

  //! Divides this Vec2d by a scalar
  Vec2d& operator/=(double ratio);

  //! Compares two Vec2d
  bool operator==(const Vec2d& other) const;

 protected:
  double x_;
  double y_;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const double ratio, const Vec2d& vec);

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_GEOMETRY_VEC2D_H_
