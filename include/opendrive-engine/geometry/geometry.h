#ifndef OPENDRIVE_ENGINE_GEOMETRY_H_
#define OPENDRIVE_ENGINE_GEOMETRY_H_

#include <string>
#include <vector>

namespace opendrive {
namespace engine {
namespace geometry {

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

class Point4D : public Point3D {
 public:
  Point4D() : Point3D(), heading_(0) {}
  Point4D(double x, double y) : Point3D(x, y), heading_(0) {}
  Point4D(double x, double y, double z) : Point3D(x, y, z), heading_(0) {}
  Point4D(double x, double y, double z, double heading)
      : Point3D(x, y, z), heading_(heading) {}
  void set_heading(double d) { heading_ = d; }
  double& mutable_heading() { return heading_; }
  double heading() const { return heading_; }

 protected:
  double heading_;
};

class Quaternion {
 public:
  Quaternion() : w_(0), x_(0), y_(0), z_(0) {}
  Quaternion(double w, double x, double y, double z)
      : w_(w), x_(x), y_(y), z_(z) {}

  void set_w(double d) { w_ = d; }
  void set_x(double d) { x_ = d; }
  void set_y(double d) { y_ = d; }
  void set_z(double d) { z_ = d; }
  double& mutable_w() { return w_; }
  double& mutable_x() { return x_; }
  double& mutable_y() { return y_; }
  double& mutable_z() { return z_; }
  double w() const { return w_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

 private:
  double w_;
  double x_;
  double y_;
  double z_;
};

}  // namespace geometry
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_GEOMETRY_H_
