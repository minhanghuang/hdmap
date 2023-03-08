#ifndef OPENDRIVE_ENGINE_CORE_GEOMETRY_H_
#define OPENDRIVE_ENGINE_CORE_GEOMETRY_H_

#include <string>
#include <vector>

namespace opendrive {
namespace engine {
namespace core {

class Point2D {
 public:
  Point2D() : x_(0), y_(0) {}
  Point2D(double x, double y) : x_(x), y_(y) {}
  void set_x(double d) { x_ = d; }
  void set_y(double d) { y_ = d; }
  double& mutable_x() { return x_; }
  double& mutable_y() { return y_; }
  double x() const { return x_; }
  double y() const { return y_; }

 protected:
  double x_;
  double y_;
};

class Point3D : public Point2D {
 public:
  Point3D() : Point2D(0, 0), z_(0) {}
  Point3D(double x, double y) : Point2D(x, y), z_(0) {}
  Point3D(double x, double y, double z) : Point2D(x, y), z_(z) {}
  void set_z(double d) { z_ = d; }
  double& mutable_z() { return z_; }
  double z() const { return z_; }

 protected:
  double z_;
};

typedef std::vector<Point3D> Polygon;

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_GEOMETRY_H_
