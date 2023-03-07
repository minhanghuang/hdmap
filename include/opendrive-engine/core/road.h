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
  RoadInfo() : start_position_(0), type_(RoadType::TOWN), speed_limit_(0) {}
  void set_s(double d) { start_position_ = d; }
  void set_type(RoadType i) { type_ = i; }
  void set_speed_limit(double d) { speed_limit_ = d; }
  double& mutable_start_position() { return start_position_; }
  RoadType& mutable_type() { return type_; }
  double& mutable_speed_limit() { return speed_limit_; }
  double start_position() const { return start_position_; }
  RoadType type() const { return type_; }
  double speed_limit() const { return speed_limit_; }

 private:
  double start_position_;
  RoadType type_;
  // meters per second
  double speed_limit_;
};
typedef std::vector<RoadInfo> RoadInfos;

class Road {
 public:
  typedef std::shared_ptr<Road> Ptr;
  typedef std::shared_ptr<Road const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  Road()
      : id_(""),
        name_(""),
        junction_id_(""),
        length_(0),
        rule_(RoadRule::RHT) {}
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
  double length_;
  RoadRule rule_;
  Section::Ptrs sections_;
  Ids predecessor_ids_;
  Ids successor_ids_;
  RoadInfos info_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_ROAD_H_
