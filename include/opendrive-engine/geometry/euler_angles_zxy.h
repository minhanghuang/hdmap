#ifndef OPENDRIVE_ENGINE_GEOMETRY_EULER_ANGLES_ZXY_H_
#define OPENDRIVE_ENGINE_GEOMETRY_EULER_ANGLES_ZXY_H_

#include <vector>

#include "opendrive-engine/geometry/geometry.h"
#include "opendrive-engine/math/math.h"

namespace opendrive {
namespace engine {
namespace geometry {

class EulerAnglesZXY {
 public:
  /**
   * @brief Constructs an identity rotation.
   */
  EulerAnglesZXY();

  /**
   * @brief Constructs a rotation using only yaw (i.e., around the z-axis).
   *
   * @param yaw The yaw of the car
   */
  explicit EulerAnglesZXY(double yaw);

  /**
   * @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
   *
   * @param roll The roll of the car
   * @param pitch The pitch of the car
   * @param yaw The yaw of the car
   */
  EulerAnglesZXY(double roll, double pitch, double yaw);

  /**
   * @brief Constructs a rotation using components of a quaternion.
   *
   * @param qw Quaternion w-coordinate
   * @param qx Quaternion x-coordinate
   * @param qy Quaternion y-coordinate
   * @param qz Quaternion z-coordinate
   */
  EulerAnglesZXY(double qw, double qx, double qy, double qz);

  /**
   * @brief Constructs a rotation from quaternion.
   * @param q Quaternion
   */
  explicit EulerAnglesZXY(const geometry::Quaternion& q);

  /**
   * @brief Getter for roll_
   * @return The roll of the car
   */
  double roll() const { return roll_; }

  /**
   * @brief Getter for pitch_
   * @return The pitch of the car
   */
  double pitch() const { return pitch_; }

  /**
   * @brief Getter for yaw_
   * @return The yaw of the car
   */
  double yaw() const { return yaw_; }

  /**
   * @brief Normalizes roll_, pitch_, and yaw_ to [-PI, PI).
   */
  void Normalize();

  /**
   * @brief Verifies the validity of the specified rotation.
   * @return True iff -PI/2 < pitch < PI/2
   */
  bool IsValid();

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation.
   */
  geometry::Quaternion ToQuaternion() const;

 private:
  double roll_;
  double pitch_;
  double yaw_;
};

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_GEOMETRY_EULER_ANGLES_ZXY_H_
