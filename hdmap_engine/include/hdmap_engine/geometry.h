#ifndef HDMAP_GEOMETRY_H_
#define HDMAP_GEOMETRY_H_

#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "hdmap_engine/common/macros.h"

namespace hdmap {
namespace geometry {

class Header {
  ADD_MEMBER_COMPLEX_TYPE(std::string, rev_major);
  ADD_MEMBER_COMPLEX_TYPE(std::string, rev_minor);
  ADD_MEMBER_COMPLEX_TYPE(std::string, name);
  ADD_MEMBER_COMPLEX_TYPE(std::string, version);
  ADD_MEMBER_COMPLEX_TYPE(std::string, date);
  ADD_MEMBER_COMPLEX_TYPE(std::string, vendor);
  ADD_MEMBER_BASIC_TYPE(double, north, 0);
  ADD_MEMBER_BASIC_TYPE(double, south, 0);
  ADD_MEMBER_BASIC_TYPE(double, west, 0);
  ADD_MEMBER_BASIC_TYPE(double, east, 0);

 public:
  using Ptr = std::shared_ptr<Header>;
  using ConstPtr = std::shared_ptr<Header const>;
  Header() {}
};

class Point2D {
  ADD_MEMBER_BASIC_TYPE(double, x, 0);
  ADD_MEMBER_BASIC_TYPE(double, y, 0);

 public:
  Point2D() : x_(0), y_(0) {}
  Point2D(double x, double y) : x_(x), y_(y) {}
};

class Point3D : public Point2D {
  ADD_MEMBER_BASIC_TYPE(double, z, 0);

 public:
  Point3D() : Point2D(0, 0), z_(0) {}
  Point3D(double x, double y) : Point2D(x, y), z_(0) {}
  Point3D(double x, double y, double z) : Point2D(x, y), z_(z) {}
};

class Point4D : public Point3D {
  ADD_MEMBER_BASIC_TYPE(double, heading, 0);

 public:
  Point4D() : Point3D(), heading_(0) {}
  Point4D(double x, double y) : Point3D(x, y), heading_(0) {}
  Point4D(double x, double y, double z) : Point3D(x, y, z), heading_(0) {}
  Point4D(double x, double y, double z, double heading)
      : Point3D(x, y, z), heading_(heading) {}
};

class Curve {
 public:
  class Point : public geometry::Point4D {
    /*
     * e.g. 207_2_-2_9_1 (size: 5)
     * road id: 270
     * section id: 270_2
     * lane id: 207_2_-2 (0: reference line; minus: left lanes; plus: right
     * lanes)
     * point: left boundary point id: 9_1 center curve point id: 9_2
     *  right boundary point id: 9_3
     */
    ADD_MEMBER_COMPLEX_TYPE(std::string, id);
    ADD_MEMBER_BASIC_TYPE(double, start_position, 0);

   public:
    Point() : start_position_(0), id_("") {}
    Point(double x, double y) : Point4D(x, y), start_position_(0), id_("") {}
    Point(double x, double y, double z)
        : Point4D(x, y, z), start_position_(0), id_("") {}
    Point(double x, double y, double z, double heading)
        : Point4D(x, y, z, heading), start_position_(0), id_("") {}
    Point(double x, double y, double z, double heading, double start_position)
        : Point4D(x, y, z, heading), start_position_(start_position), id_("") {}
    Point(double x, double y, double z, double heading, double start_position,
          const std::string& id)
        : Point4D(x, y, z, heading), start_position_(start_position), id_(id) {}
  };
  using Points = std::vector<Point>;
  using Line = std::vector<Point>;
  ADD_MEMBER_COMPLEX_TYPE(Line, pts);
  ADD_MEMBER_BASIC_TYPE(double, length, 0);
};

class Lane {
 public:
  enum class Type {
    kDriving = 1,
  };
  class Speed {
    ADD_MEMBER_BASIC_TYPE(double, start_position, 0);
    ADD_MEMBER_BASIC_TYPE(float, data, 0);

   public:
    Speed() : start_position_(0), data_(0) {}
  };
  class Boundary {
   public:
    enum class Color {
      kWhite = 0,
      kYellow,
    };
    enum class Type {
      kBroken = 0,
      kSolid,
    };
    ADD_MEMBER_COMPLEX_TYPE(Curve, curve);
    ADD_MEMBER_COMPLEX_TYPE(Color, color);
    ADD_MEMBER_COMPLEX_TYPE(Type, type);

   public:
    Boundary() : color_(Color::kWhite), type_(Type::kBroken) {}
  };

  ADD_MEMBER_COMPLEX_TYPE(std::string, id);
  ADD_MEMBER_COMPLEX_TYPE(std::string, parent_id);
  ADD_MEMBER_COMPLEX_TYPE(std::string, predecessor_ids);
  ADD_MEMBER_COMPLEX_TYPE(std::string, successor_ids);
  ADD_MEMBER_COMPLEX_TYPE(Curve, central_curve);
  ADD_MEMBER_COMPLEX_TYPE(Boundary, left_boundary);
  ADD_MEMBER_COMPLEX_TYPE(Boundary, right_boundary);

 public:
  using Ptr = std::shared_ptr<Lane>;
  using ConstPtr = std::shared_ptr<Lane const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Lane() {}
};

class Section {
  ADD_MEMBER_COMPLEX_TYPE(std::string, id);
  ADD_MEMBER_COMPLEX_TYPE(std::string, parent_id);
  ADD_MEMBER_BASIC_TYPE(double, start_position, 0);
  ADD_MEMBER_BASIC_TYPE(double, end_position, 0);
  ADD_MEMBER_BASIC_TYPE(double, length, 0);
  ADD_MEMBER_SHAREDPTR_TYPE(Lane, center_lane);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<Lane::Ptr>, left_lanes);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<Lane::Ptr>, right_lanes);

 public:
  using Ptr = std::shared_ptr<Section>;
  using ConstPtr = std::shared_ptr<Section const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Section() {}
};

class Road {
  ADD_MEMBER_COMPLEX_TYPE(std::string, id);
  ADD_MEMBER_COMPLEX_TYPE(std::string, name);
  ADD_MEMBER_BASIC_TYPE(double, length, 0);
  ADD_MEMBER_COMPLEX_TYPE(Section::Ptrs, sections);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<std::string>, predecessor_ids);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<std::string>, successor_ids);

 public:
  using Ptr = std::shared_ptr<Road>;
  using ConstPtr = std::shared_ptr<Road const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Road() {}
};

using LaneRoute = std::unordered_map<std::string, Lane::Ptr>;
using SectionRoute = std::unordered_map<std::string, Section::Ptr>;
using RoadRoute = std::unordered_map<std::string, Road::Ptr>;

class Map {
  ADD_MEMBER_SHAREDPTR_TYPE(Header, header);
  ADD_MEMBER_COMPLEX_TYPE(LaneRoute, lanes);
  ADD_MEMBER_COMPLEX_TYPE(SectionRoute, sections);
  ADD_MEMBER_COMPLEX_TYPE(RoadRoute, roads);

 public:
  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<Map const>;
  Map() {}
};

}  // namespace geometry
}  // namespace hdmap

#endif  // HDMAP_GEOMETRY_H_
