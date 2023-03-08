#ifndef OPENDRIVE_ENGINE_DEFINE_H_
#define OPENDRIVE_ENGINE_DEFINE_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "header.h"
#include "id.h"
#include "junction.h"
#include "lane.h"
#include "road.h"
#include "section.h"

namespace opendrive {
namespace engine {
namespace core {

typedef std::unordered_map<Id, Lane::Ptr> LaneRoute;
typedef std::unordered_map<Id, Lane::ConstPtr> ConstLaneRoute;
typedef std::unordered_map<Id, Section::Ptr> SectionRoute;
typedef std::unordered_map<Id, Section::ConstPtr> ConstSectionRoute;
typedef std::unordered_map<Id, Road::Ptr> RoadRoute;
typedef std::unordered_map<Id, Road::ConstPtr> ConstRoadRoute;
typedef std::unordered_map<Id, Junction::Ptr> JunctionRoute;
typedef std::unordered_map<Id, Junction::ConstPtr> ConstJunctionRoute;

class Data {
 public:
  typedef std::shared_ptr<Data> Ptr;
  typedef std::shared_ptr<Data const> ConstPtr;
  Data() {}
  void set_header(Header::Ptr p) { header_ = p; }
  void set_lanes(const LaneRoute& m) { lanes_ = m; }
  void set_sections(const SectionRoute& m) { sections_ = m; }
  void set_roads(const RoadRoute& m) { roads_ = m; }
  void set_junctions(const JunctionRoute& m) { junctions_ = m; }
  Header::Ptr mutable_header() { return header_; }
  LaneRoute& mutable_lanes() { return lanes_; }
  SectionRoute& mutable_sections() { return sections_; }
  RoadRoute& mutable_roads() { return roads_; }
  JunctionRoute& mutable_junction() { return junctions_; }
  Header::ConstPtr header() const { return header_; }
  const LaneRoute& lanes() const { return lanes_; }
  const SectionRoute& sections() const { return sections_; }
  const RoadRoute& roads() const { return roads_; }
  const JunctionRoute& junctions() const { return junctions_; }

 private:
  Header::Ptr header_;
  LaneRoute lanes_;
  SectionRoute sections_;
  RoadRoute roads_;
  JunctionRoute junctions_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_DEFINE_H_
