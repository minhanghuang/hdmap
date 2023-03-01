#ifndef OPENDRIVE_ENGINE_CORE_ROAD_H_
#define OPENDRIVE_ENGINE_CORE_ROAD_H_

#include <opendrive-cpp/geometry/enums.h>

#include <vector>

#include "id.h"
#include "section.h"

namespace opendrive {
namespace engine {
namespace core {

class RoadInfo {
 public:
  void set_s(double d) { s_ = d; }
  void set_type(RoadType t) { type_ = t; }
  void set_speed_limit(double d) { speed_limit_ = d; }
  double s() const { return s_; }
  RoadType type() const { return type_; }
  double speed_limit() const { return speed_limit_; }

 private:
  double s_ = 0.;
  RoadType type_ = RoadType::TOWN;
  double speed_limit_;  // meters per second
};
typedef std::vector<RoadInfo> RoadInfos;

class Road {
 public:
  typedef std::shared_ptr<Road> Ptr;
  typedef std::shared_ptr<Road const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  void set_id(const Id& s) { id_ = s; }
  void set_name(const std::string& s) { name_ = s; }
  void set_junction_id(const Id& s) { junction_id_ = "-1" == s ? "" : s; }
  void set_length(double d) { length_ = d; }
  Section::Ptrs& mutable_sections() { return sections_; }
  Ids& mutable_predecessor_ids() { return predecessor_ids_; }
  Ids& mutable_successor_ids() { return successor_ids_; }
  void set_rule(RoadRule t) { rule_ = t; }
  RoadInfos& mutable_info() { return info_; }
  const Id& id() const { return id_; }
  const std::string& name() const { return name_; }
  const Id& junction_id() const { return junction_id_; }
  double length() const { return length_; }
  Section::ConstPtrs sections() const {
    Section::ConstPtrs ret;
    for (const auto& section : sections_) {
      ret.emplace_back(section);
    }
    return ret;
  }
  const Ids& predecessor_ids() const { return predecessor_ids_; }
  const Ids& successor_ids() const { return successor_ids_; }
  RoadRule rule() const { return rule_; }
  const RoadInfos& info() const { return info_; }

 private:
  Id id_;
  std::string name_;
  Id junction_id_;
  double length_ = 0;
  Section::Ptrs sections_;
  Ids predecessor_ids_;
  Ids successor_ids_;
  RoadRule rule_ = RoadRule::RHT;
  RoadInfos info_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_ROAD_H_
