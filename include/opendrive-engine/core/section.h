#ifndef OPENDRIVE_ENGINE_CORE_SECTION_H_
#define OPENDRIVE_ENGINE_CORE_SECTION_H_

#include <memory>
#include <vector>

#include "id.h"
#include "lane.h"

namespace opendrive {
namespace engine {
namespace core {

class Section {
 public:
  typedef std::shared_ptr<Section> Ptr;
  typedef std::shared_ptr<Section const> ConstPtr;
  typedef std::vector<Ptr> Ptrs;
  typedef std::vector<ConstPtr> ConstPtrs;
  Section()
      : id_(""),
        parent_id_(""),
        start_position_(0),
        end_position_(0),
        length_(0) {}
  void set_id(const Id& s) { id_ = s; }
  void set_parent_id(const Id& s) { parent_id_ = s; }
  void set_start_position(double d) { start_position_ = d; }
  void set_end_position(double d) { end_position_ = d; }
  void set_length(double d) { length_ = d; }
  void set_center_lane(Lane::Ptr p) { center_lane_ = p; }
  void set_left_lanes(Lane::Ptrs v) { left_lanes_ = v; }
  void set_right_lanes(Lane::Ptrs v) { right_lanes_ = v; }
  Id& mutable_id() { return id_; }
  Id& mutable_parent_id() { return parent_id_; }
  double& mutable_start_position() { return start_position_; }
  double& mutable_end_position() { return end_position_; }
  double& mutable_length() { return length_; }
  Lane::Ptr& mutable_center_lane() { return center_lane_; }
  Lane::Ptrs& mutable_left_lanes() { return left_lanes_; }
  Lane::Ptrs& mutable_right_lanes() { return right_lanes_; }
  const Id& id() const { return id_; }
  const Id& parent_id() const { return parent_id_; }
  double start_position() const { return start_position_; }
  double end_position() const { return end_position_; }
  double length() const { return length_; }
  Lane::ConstPtr center_lane() const { return center_lane_; }
  Lane::ConstPtrs left_lanes() const {
    Lane::ConstPtrs lanes;
    for (const auto& lane : left_lanes_) lanes.emplace_back(lane);
    return lanes;
  }
  Lane::ConstPtrs right_lanes() const {
    Lane::ConstPtrs lanes;
    for (const auto& lane : right_lanes_) lanes.emplace_back(lane);
    return lanes;
  }

 private:
  Id id_;
  Id parent_id_;  // road id
  double start_position_ = 0;
  double end_position_ = 0;
  double length_ = 0;
  Lane::Ptr center_lane_;
  Lane::Ptrs left_lanes_;
  Lane::Ptrs right_lanes_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_SECTION_H_
