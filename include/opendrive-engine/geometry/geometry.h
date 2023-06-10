#ifndef OPENDRIVE_ENGINE_GEOMETRY_H_
#define OPENDRIVE_ENGINE_GEOMETRY_H_

#include <cactus/macros.h>

#include <string>
#include <vector>

namespace opendrive {
namespace engine {
namespace geometry {

class Point2D {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, x, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, y, 0);

 public:
  Point2D() : x_(0), y_(0) {}
  Point2D(double x, double y) : x_(x), y_(y) {}
};

class Point3D : public Point2D {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, z, 0);

 public:
  Point3D() : Point2D(0, 0), z_(0) {}
  Point3D(double x, double y) : Point2D(x, y), z_(0) {}
  Point3D(double x, double y, double z) : Point2D(x, y), z_(z) {}
};

class Point4D : public Point3D {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, heading, 0);

 public:
  Point4D() : Point3D(), heading_(0) {}
  Point4D(double x, double y) : Point3D(x, y), heading_(0) {}
  Point4D(double x, double y, double z) : Point3D(x, y, z), heading_(0) {}
  Point4D(double x, double y, double z, double heading)
      : Point3D(x, y, z), heading_(heading) {}
};

class Quaternion {
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, qw, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, qx, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, qy, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, qz, 0);

 public:
  Quaternion() {}
  Quaternion(double w, double x, double y, double z)
      : qw_(w), qx_(x), qy_(y), qz_(z) {}
};

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_GEOMETRY_H_
