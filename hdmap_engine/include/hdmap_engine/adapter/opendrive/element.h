#ifndef HDMAP_ENGINE_ADAPTER_OPENDRIVE_ELEMENT_H_
#define HDMAP_ENGINE_ADAPTER_OPENDRIVE_ELEMENT_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hdmap_common/util.h"
#include "hdmap_engine/3rd_party/odrSpiral/odrSpiral.h"
#include "hdmap_engine/common/macros.h"

namespace hdmap {
namespace opendrive {

namespace enums {

enum class Boolean : std::uint8_t { kFalse = 0, kTrue, kUnknown };

enum class GeometryType : std::uint8_t {
  kArc,
  kLine,
  kSpiral,
  kPoly3,
  kParamPoly3,
};

enum class LaneType : std::uint8_t {
  kSholder = 0,  // Describes a soft border at the edge of the road.
  kBorder,   // Describes a hard border at the edge of the road. It has the same
             // height as the drivable lane.
  kDriving,  // Describes a "normal" drivable road that is not one of the other
             // types.
  kStop,     // Hard shoulder on motorways for emergency stops
  kNone,  // Describes the space on the outermost edge of the road and does not
          // have actual content Its only purpose is for applications to
          // register that ASAM OpenDRIVE is still present in case the (human)
          // driver leaves the road.
  kRestricted,  // Describes a lane on which cars should not drive. The lane has
                // the same height as drivable lanes. Typically, the lane is
                // separated with lines and often contains dotted lines as well.
  kParking,     // Describes a lane with parking spaces.
  kMedian,      // Describes a lane that sits between driving lanes that lead in
            // opposite directions. It is typically used to separate traffic in
            // towns on large roads.
  kBiking,    // Describes a lane that is reserved for cyclists.
  kSidewalk,  // Describes a lane on which pedestrians can walk.
  kCurb,  // Describes curb stones. Curb stones have a different height than the
          // adjacent drivable lanes.
  kExit,  // Describes a lane that is used for sections that are parallel to the
          // main road. It is mainly used for deceleration lanes.
  kEntry,   // Describes a lane type that is used for sections that are parallel
            // to the main road. It is mainly used for acceleration lanes.
  kOnramp,  // A ramp leading to a motorway from rural or urban roads.
  kOfframp,  //  A ramp leading away from a motorway and onto rural urban roads.
  kConnectingramp,  //  A ramp that connects two motorways, for example,
                    //  motorway junctions.
  kBidirectional,
  kSpecial1,
  kSpecial2,
  kSpecial3,
  kRoadworks,
  kTram,
  kRail,
  kBus,
  kTaxi,
  kHov,
  kMwyentry,
  kMwyexit
};

enum class RoadMarkType : std::uint8_t {
  kUnknown = 0,
  kNone,
  kSolid,
  kBroken,
  kSolidsolid,
  kSolidbroken,
  kBrokensolid,
  kBrokenbroken,
  kBottsdots,
  kGrass,
  kCurb,
  kCustom,
  kEdge
};

enum class RoadMarkColor : std::uint8_t {
  kStandard = 0,
  kBlue,
  kGreen,
  kRed,
  kWhite,
  kYellow,
  kOrange
};

enum class RoadMarkWeight : std::uint8_t { kUnknown = 0, kStandard, kBold };

enum class RoadMarkLaneChange : std::uint8_t {
  kUnknown = 0,
  kNone,
  kIncrease,
  kDecrease,
  kBoth,
};

enum class RoadRule { kRht, kLht };

enum class RoadType : std::uint8_t {
  kRural = 0,
  kMotorway,
  kTown,
  kLowspeed,
  kPedestrian,
  kBicycle,
  kTownexpressway,
  kTowncollector,
  kTownarterial,
  kTownprivate,
  kTownlocal,
  kTownplaystreet,
};

enum class RoadLinkType { kRoad = 0, kJunction };

enum class SpeedUnit {
  kMs = 0,  // m/s
  kMph,     // mph
  kKmh      // km/h
};

enum class LaneDirection { kUnknown = 0, kLeft, kCenter, kRight };

enum class JunctionType { kDefault, kDirect, kVirtual };

enum class JunctionConnectionType { kUnknown = 0, kDefault, kVirtual };

enum class ContactPointType { kUnknown = 0, kStart, kEnd };

enum class Dir { kUnknown = 0, kPlus, kMinus };

}  // namespace enums

namespace element {

using Id = int;
using Idx = Id;
using Ids = std::vector<Id>;
using IdStr = std::string;
using Name = std::string;

class Point {
  ADD_MEMBER_BASIC_TYPE(double, x, 0);
  ADD_MEMBER_BASIC_TYPE(double, y, 0);
  ADD_MEMBER_BASIC_TYPE(double, z, 0);
  ADD_MEMBER_BASIC_TYPE(double, heading, 0);

 public:
  Point() : x_(0), y_(0), z_(0), heading_(0) {}
  Point(double x, double y, double z) : x_(x), y_(y), z_(z), heading_(0) {}
  Point(double x, double y, double z, double heading)
      : x_(x), y_(y), z_(z), heading_(heading) {}
};

class Header {
  ADD_MEMBER_COMPLEX_TYPE(std::string, rev_major);
  ADD_MEMBER_COMPLEX_TYPE(std::string, rev_minor);
  ADD_MEMBER_COMPLEX_TYPE(std::string, version);
  ADD_MEMBER_COMPLEX_TYPE(std::string, name);
  ADD_MEMBER_COMPLEX_TYPE(std::string, date);
  ADD_MEMBER_COMPLEX_TYPE(std::string, vendor);
  ADD_MEMBER_BASIC_TYPE(double, north, 0);
  ADD_MEMBER_BASIC_TYPE(double, south, 0);
  ADD_MEMBER_BASIC_TYPE(double, west, 0);
  ADD_MEMBER_BASIC_TYPE(double, east, 0);

 public:
  Header() : north_(0), south_(0), west_(0), east_(0) {}
};

class Geometry {
  ADD_MEMBER_BASIC_TYPE(double, s, 0);
  ADD_MEMBER_BASIC_TYPE(double, x, 0);
  ADD_MEMBER_BASIC_TYPE(double, y, 0);
  ADD_MEMBER_BASIC_TYPE(double, hdg, 0);
  ADD_MEMBER_BASIC_TYPE(double, length, 0);
  ADD_MEMBER_BASIC_TYPE(double, sin_hdg, 0);
  ADD_MEMBER_BASIC_TYPE(double, cos_hdg, 0);
  ADD_MEMBER_COMPLEX_TYPE(enums::GeometryType, type);

 public:
  using Ptr = std::shared_ptr<Geometry>;
  using ConstPtr = std::shared_ptr<Geometry const>;
  using Ptrs = std::vector<Ptr>;
  using ConstPtrs = std::vector<ConstPtr>;
  Geometry(double _s, double _x, double _y, double _hdg, double _length,
           enums::GeometryType _type)
      : s_(_s),
        x_(_x),
        y_(_y),
        hdg_(_hdg),
        length_(_length),
        type_(_type),
        sin_hdg_(std::sin(_hdg)),
        cos_hdg_(std::cos(_hdg)) {}
  virtual ~Geometry() = default;
  virtual Point GetPoint(double ref_line_ds) const = 0;
};

class GeometryLine final : public Geometry {
 public:
  GeometryLine(double s, double x, double y, double hdg, double length,
               enums::GeometryType type)
      : Geometry(s, x, y, hdg, length, type) {}
  virtual Point GetPoint(double road_ds) const override {
    const double ref_line_ds = road_ds - s();
    const double xd = x() + (cos_hdg() * ref_line_ds);
    const double yd = y() + (sin_hdg() * ref_line_ds);
    return Point{xd, yd, 0, hdg()};
  }
};

class GeometryArc final : public Geometry {
  ADD_MEMBER_BASIC_TYPE(double, curvature, 0);
  ADD_MEMBER_BASIC_TYPE(double, radius, 0);

 public:
  GeometryArc(double s, double x, double y, double hdg, double length,
              enums::GeometryType type, double curvature)
      : Geometry(s, x, y, hdg, length, type),
        curvature_(curvature),
        radius_(1.0 / curvature) {}
  virtual Point GetPoint(double road_ds) const override {
    const double ref_line_ds = road_ds - s();
    const double angle_at_s = ref_line_ds * curvature_ - M_PI / 2;
    const double xd =
        radius_ * (std::cos(hdg() + angle_at_s) - sin_hdg()) + x();
    const double yd =
        radius_ * (std::sin(hdg() + angle_at_s) + cos_hdg()) + y();
    const double tangent = hdg() + ref_line_ds * curvature_;
    return Point{xd, yd, 0, tangent};
  }
};

class GeometrySpiral final : public Geometry {
  ADD_MEMBER_BASIC_TYPE(double, curve_start, 0);
  ADD_MEMBER_BASIC_TYPE(double, curve_end, 0);
  ADD_MEMBER_BASIC_TYPE(double, curve_dot, 0);

 public:
  GeometrySpiral(double s, double x, double y, double hdg, double length,
                 enums::GeometryType type, double curve_start, double curve_end)
      : Geometry(s, x, y, hdg, length, type),
        curve_start_(curve_start),
        curve_end_(curve_end),
        curve_dot_((curve_end - curve_start) / (length)) {}

  virtual Point GetPoint(double road_ds) const override {
    const double ref_line_ds = road_ds - s();
    const double s1 = curve_start_ / curve_dot_ + ref_line_ds;
    double x1;
    double y1;
    double t1;
    odrSpiral(s1, curve_dot_, &x1, &y1, &t1);

    const double s0 = curve_start_ / curve_dot_;
    double x0;
    double y0;
    double t0;
    odrSpiral(s1, curve_dot_, &x0, &y0, &t0);

    x1 -= x0;
    y1 -= y0;
    t1 -= t0;
    const double angle = hdg() - t0;
    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    const double xd = x() + x1 * cos_a - y1 * sin_a;
    const double yd = y() + y1 * cos_a + x1 * sin_a;
    const double tangent = hdg() + t1;
    return Point{xd, yd, 0, tangent};
  }
};

class GeometryPoly3 final : public Geometry {
  ADD_MEMBER_BASIC_TYPE(double, a, 0);
  ADD_MEMBER_BASIC_TYPE(double, b, 0);
  ADD_MEMBER_BASIC_TYPE(double, c, 0);
  ADD_MEMBER_BASIC_TYPE(double, d, 0);

 public:
  GeometryPoly3(double s, double x, double y, double hdg, double length,
                enums::GeometryType type, double a, double b, double c,
                double d)
      : Geometry(s, x, y, hdg, length, type), a_(a), b_(b), c_(c), d_(d) {}

  virtual Point GetPoint(double road_ds) const override {
    const double ref_line_ds = road_ds - s();
    const double u = ref_line_ds;
    const double v = a_ + b_ * u + c_ * std::pow(u, 2) + d_ * std::pow(u, 3);
    const double x1 = u * cos_hdg() - v * sin_hdg();
    const double y1 = u * sin_hdg() + v * cos_hdg();
    const double tangent_v = b_ + 2.0 * c_ * u + 3.0 * d_ * std::pow(u, 2);
    const double theta = std::atan2(tangent_v, 1.0);
    const double xd = x() + x1;
    const double yd = y() + y1;
    const double tangent = hdg() + theta;
    return Point{xd, yd, 0, tangent};
  }
};

class GeometryParamPoly3 final : public Geometry {
 public:
  enum class PRange : std::uint8_t {
    UNKNOWN = 0,
    ARCLENGTH = 1,
    NORMALIZED = 2
  };
  ADD_MEMBER_COMPLEX_TYPE(PRange, p_range);
  ADD_MEMBER_BASIC_TYPE(double, au, 0);
  ADD_MEMBER_BASIC_TYPE(double, bu, 0);
  ADD_MEMBER_BASIC_TYPE(double, cu, 0);
  ADD_MEMBER_BASIC_TYPE(double, du, 0);
  ADD_MEMBER_BASIC_TYPE(double, av, 0);
  ADD_MEMBER_BASIC_TYPE(double, bv, 0);
  ADD_MEMBER_BASIC_TYPE(double, cv, 0);
  ADD_MEMBER_BASIC_TYPE(double, dv, 0);

 public:
  GeometryParamPoly3(double s, double x, double y, double hdg, double length,
                     enums::GeometryType type, double au, double bu, double cu,
                     double du, double av, double bv, double cv, double dv,
                     PRange p_range)
      : Geometry(s, x, y, hdg, length, type),
        au_(au),
        bu_(bu),
        cu_(cu),
        du_(du),
        av_(av),
        bv_(bv),
        cv_(cv),
        dv_(dv),
        p_range_(p_range) {}
  virtual Point GetPoint(double road_ds) const override {
    const double ref_line_ds = road_ds - s();
    double p = ref_line_ds;
    if (PRange::NORMALIZED == p_range_) {
      p = std::min(1.0, ref_line_ds / length());
    }
    const double u =
        au_ + bu_ * p + cu_ * std::pow(p, 2) + du_ * std::pow(p, 3);
    const double v =
        av_ + bv_ * p + cv_ * std::pow(p, 2) + dv_ * std::pow(p, 3);
    const double x1 = u * cos_hdg() - v * sin_hdg();
    const double y1 = u * sin_hdg() + v * cos_hdg();
    const double tangent_u = bu_ + 2 * cu_ * p + 3 * du_ * std::pow(p, 2);
    const double tangent_v = bv_ + 2 * cv_ * p + 3 * dv_ * std::pow(p, 2);
    const double theta = std::atan2(tangent_v, tangent_u);
    const double xd = x() + x1;
    const double yd = y() + y1;
    const double tangent = hdg() + theta;
    return Point{xd, yd, 0, tangent};
  }
};

class LaneAttribute {
  ADD_MEMBER_BASIC_TYPE(Id, id, std::numeric_limits<Id>::max());
  ADD_MEMBER_COMPLEX_TYPE(enums::LaneType, type);
  ADD_MEMBER_COMPLEX_TYPE(enums::Boolean, level);

 public:
  LaneAttribute()
      : id_(std::numeric_limits<Id>::max()),
        type_(enums::LaneType::kDriving),
        level_(enums::Boolean::kUnknown) {}
};

class OffsetPoly3 {
  // f(s) = a + b*s + c*s*s + d*s*s*s
  ADD_MEMBER_BASIC_TYPE(double, s, 0);
  ADD_MEMBER_BASIC_TYPE(double, a, 0);
  ADD_MEMBER_BASIC_TYPE(double, b, 0);
  ADD_MEMBER_BASIC_TYPE(double, c, 0);
  ADD_MEMBER_BASIC_TYPE(double, d, 0);

 public:
  OffsetPoly3() : s_(0), a_(0), b_(0), c_(0), d_(0) {}
  bool operator<(const OffsetPoly3& obj) const { return this->s_ > obj.s_; }
  virtual double GetOffsetValue(double road_ds) const final {
    const double ds = road_ds - s_;
    return a_ + b_ * ds + c_ * std::pow(ds, 2) + d_ * std::pow(ds, 3);
  }
};

class RoadMark {
  ADD_MEMBER_BASIC_TYPE(double, s, 0);
  ADD_MEMBER_COMPLEX_TYPE(enums::RoadMarkType, type);
  ADD_MEMBER_COMPLEX_TYPE(enums::RoadMarkColor, color);
  ADD_MEMBER_COMPLEX_TYPE(enums::RoadMarkWeight, weight);
  ADD_MEMBER_COMPLEX_TYPE(enums::RoadMarkLaneChange, lane_change);
  ADD_MEMBER_BASIC_TYPE(double, width, 0);
  ADD_MEMBER_BASIC_TYPE(double, height, 0);
  ADD_MEMBER_COMPLEX_TYPE(std::string, material);

 public:
  RoadMark()
      : s_(0),
        type_(enums::RoadMarkType::kNone),
        color_(enums::RoadMarkColor::kStandard),
        weight_(enums::RoadMarkWeight::kUnknown),
        lane_change_(enums::RoadMarkLaneChange::kUnknown),
        width_(0),
        height_(0),
        material_("standard") {}
};
using RoadMarks = std::vector<RoadMark>;

class LaneWidth : public OffsetPoly3 {};
using LaneWidths = std::vector<LaneWidth>;

class LaneBorder : public OffsetPoly3 {};
using LaneBorders = std::vector<LaneBorder>;

class LaneLink {
  ADD_MEMBER_COMPLEX_TYPE(Ids, predecessors);
  ADD_MEMBER_COMPLEX_TYPE(Ids, successors);

 public:
  LaneLink() {}
};
using LaneLinks = std::vector<LaneLink>;

class LaneSpeed {
  ADD_MEMBER_BASIC_TYPE(double, s, 0);
  ADD_MEMBER_BASIC_TYPE(float, max, 0);
  ADD_MEMBER_COMPLEX_TYPE(enums::SpeedUnit, unit);

 public:
  LaneSpeed() : s_(0), max_(0), unit_(enums::SpeedUnit::kMs) {}
};
using LaneSpeeds = std::vector<LaneSpeed>;

class Lane {
  ADD_MEMBER_COMPLEX_TYPE(LaneAttribute, attribute);
  ADD_MEMBER_COMPLEX_TYPE(LaneLink, link);
  ADD_MEMBER_COMPLEX_TYPE(LaneWidths, widths);
  ADD_MEMBER_COMPLEX_TYPE(LaneBorders, borders);
  ADD_MEMBER_COMPLEX_TYPE(RoadMarks, road_marks);
  ADD_MEMBER_COMPLEX_TYPE(LaneSpeeds, max_speeds);

 public:
  Lane() {}
  double GetLaneWidth(double road_ds) const {
    // width >> border
    if (road_ds < 0) {
      return GetLaneWidth(0.);
    }
    if (widths_.empty()) {
      if (borders_.empty()) {
        return 0.;
      }
      /// border
      int border_index = common::GetGtValuePoloy3(borders_, road_ds);
      if (border_index < 0) {
        return 0.;
      }
      auto border = widths_.at(border_index);
      return border.GetOffsetValue(road_ds);
    } else {
      /// width
      int width_index = common::GetGtValuePoloy3(widths_, road_ds);
      if (width_index < 0) {
        return 0.;
      }
      auto width = widths_.at(width_index);
      // std::cout << "sssss: " << road_ds << "   " << width.a() << " "
      //           << width.b() << "  " << width.c() << "  " << width.d() << " "
      //           << width.s() << std::endl;
      return width.GetOffsetValue(road_ds);
    }
    return 0.;
  }
};

class LanesInfo {
  ADD_MEMBER_COMPLEX_TYPE(std::vector<Lane>, lanes);

 public:
  LanesInfo() {}
};

class LaneOffset : public OffsetPoly3 {};
using LaneOffsets = std::vector<LaneOffset>;

class LaneSection {
  ADD_MEMBER_BASIC_TYPE(Id, id, -1);
  ADD_MEMBER_BASIC_TYPE(double, start_position, 0);
  ADD_MEMBER_BASIC_TYPE(double, end_position, 0);
  ADD_MEMBER_COMPLEX_TYPE(LanesInfo, left);
  ADD_MEMBER_COMPLEX_TYPE(LanesInfo, center);
  ADD_MEMBER_COMPLEX_TYPE(LanesInfo, right);

 public:
  LaneSection() : id_(-1), start_position_(0), end_position_(0) {}
};
using LaneSections = std::vector<LaneSection>;

class Lanes {
  ADD_MEMBER_COMPLEX_TYPE(LaneOffsets, lane_offsets);
  ADD_MEMBER_COMPLEX_TYPE(LaneSections, lane_sections);

 public:
  Lanes() {}
};

class RoadAttribute {
  ADD_MEMBER_COMPLEX_TYPE(Name, name);
  ADD_MEMBER_BASIC_TYPE(Id, id, -1);  // [>=0]
  ADD_MEMBER_BASIC_TYPE(Id, junction_id,
                        -1);  // -1: Road; other: junction road
  ADD_MEMBER_BASIC_TYPE(double, length, 0);
  ADD_MEMBER_COMPLEX_TYPE(
      enums::RoadRule, rule);  // RHT=right-hand traffic, LHT=left-hand traffic.
                               // When this attribute is missing, RHT is assumed

 public:
  RoadAttribute()
      : name_(""),
        id_(-1),
        junction_id_(-1),
        length_(0),
        rule_(enums::RoadRule::kRht) {}
};

class RoadLinkInfo {
  ADD_MEMBER_BASIC_TYPE(Id, id, -1);  // [>=0]
  ADD_MEMBER_BASIC_TYPE(double, start_position, -1);
  ADD_MEMBER_COMPLEX_TYPE(enums::RoadLinkType, type);
  ADD_MEMBER_COMPLEX_TYPE(
      enums::ContactPointType,
      contact_point);  // Contact point of link on the linked element
  ADD_MEMBER_COMPLEX_TYPE(enums::Dir, dir);

 public:
  RoadLinkInfo()
      : id_(-1),
        start_position_(-1),
        type_(enums::RoadLinkType::kRoad),
        contact_point_(enums::ContactPointType::kUnknown),
        dir_(enums::Dir::kUnknown) {}
};

class RoadLink {
  ADD_MEMBER_COMPLEX_TYPE(RoadLinkInfo, predecessor);
  ADD_MEMBER_COMPLEX_TYPE(RoadLinkInfo, successor);

 public:
  RoadLink() {}
};

class RoadTypeInfo {
  ADD_MEMBER_BASIC_TYPE(double, start_position, -1);
  ADD_MEMBER_COMPLEX_TYPE(enums::RoadType, type);
  ADD_MEMBER_COMPLEX_TYPE(std::string, country);
  ADD_MEMBER_BASIC_TYPE(float, max_speed, 0);
  ADD_MEMBER_COMPLEX_TYPE(enums::SpeedUnit, speed_unit);

 public:
  RoadTypeInfo()
      : start_position_(0),
        type_(enums::RoadType::kTown),
        country_(""),
        max_speed_(0),
        speed_unit_(enums::SpeedUnit::kMs) {}
};

class RoadPlanView {
  ADD_MEMBER_COMPLEX_TYPE(Geometry::Ptrs, geometrys);

 public:
  RoadPlanView() {}
};

class Road {
  ADD_MEMBER_COMPLEX_TYPE(RoadAttribute, attribute);
  ADD_MEMBER_COMPLEX_TYPE(RoadLink, link);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<RoadTypeInfo>, type_info);
  ADD_MEMBER_COMPLEX_TYPE(RoadPlanView, plan_view);
  ADD_MEMBER_COMPLEX_TYPE(Lanes, lanes);

 public:
  Road() {}
};

class JunctionAttribute {
  ADD_MEMBER_BASIC_TYPE(Id, id, -1);
  ADD_MEMBER_COMPLEX_TYPE(Name, name);
  ADD_MEMBER_COMPLEX_TYPE(enums::JunctionType, type);
  ADD_MEMBER_BASIC_TYPE(Id, main_road, -1);  // virtual junctions v1.7
  ADD_MEMBER_BASIC_TYPE(double, start_position,
                        -1);  // virtual junctions v1.7
  ADD_MEMBER_BASIC_TYPE(double, end_position,
                        -1);                 // virtual junctions v1.7
  ADD_MEMBER_COMPLEX_TYPE(enums::Dir, dir);  // virtual junctions v1.7

 public:
  JunctionAttribute()
      : id_(-1),
        type_(enums::JunctionType::kDefault),
        main_road_(-1),
        start_position_(-1),
        end_position_(-1),
        dir_(enums::Dir::kUnknown) {}
};

class JunctionLaneLink {
  ADD_MEMBER_BASIC_TYPE(Id, from, -1);
  ADD_MEMBER_BASIC_TYPE(Id, to, -1);

 public:
  JunctionLaneLink() : from_(-1), to_(-1) {}
};
using JunctionLaneLinks = std::vector<JunctionLaneLink>;

class JunctionConnection {
  ADD_MEMBER_BASIC_TYPE(Id, id, -1);
  ADD_MEMBER_COMPLEX_TYPE(enums::JunctionConnectionType, type);
  ADD_MEMBER_BASIC_TYPE(Id, incoming_road, -1);
  ADD_MEMBER_BASIC_TYPE(Id, connecting_road, -1);
  ADD_MEMBER_BASIC_TYPE(Id, linked_road, -1);
  ADD_MEMBER_COMPLEX_TYPE(enums::ContactPointType, contact_point);
  ADD_MEMBER_COMPLEX_TYPE(JunctionLaneLinks, lane_links);

 public:
  JunctionConnection()
      : id_(-1),
        type_(enums::JunctionConnectionType::kUnknown),
        incoming_road_(-1),
        connecting_road_(-1),
        linked_road_(-1),
        contact_point_(enums::ContactPointType::kUnknown) {}
};
using JunctionConnections = std::vector<JunctionConnection>;

class Junction {
  ADD_MEMBER_COMPLEX_TYPE(JunctionAttribute, attribute);
  ADD_MEMBER_COMPLEX_TYPE(JunctionConnections, connections);

 public:
  Junction() {}
};

class Map {
  ADD_MEMBER_COMPLEX_TYPE(Header, header);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<Road>, roads);
  ADD_MEMBER_COMPLEX_TYPE(std::vector<Junction>, junctions);

 public:
  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<Map const>;
  Map() {}
};

}  // namespace element
}  // namespace opendrive
}  // namespace hdmap

#endif  // HDMAP_ENGINE_ADAPTER_OPENDRIVE_ELEMENT_H_
