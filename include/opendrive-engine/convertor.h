#ifndef OPENDRIVE_ENGINE_CONVERTOR_H_
#define OPENDRIVE_ENGINE_CONVERTOR_H_

#include <cactus/cactus.h>

#include <memory>
#include <string>

#include "opendrive-cpp/geometry/element.h"
#include "opendrive-engine/algo/kdtree/kdtree.h"
#include "opendrive-engine/common/log.h"
#include "opendrive-engine/common/param.h"
#include "opendrive-engine/common/status.h"
#include "opendrive-engine/core/define.h"
#include "opendrive-engine/core/lane.h"

namespace opendrive {
namespace engine {

class Convertor {
 public:
  Convertor() = default;
  Status Start();

 private:
  inline void SetStatus(ErrorCode code, const std::string& msg);
  inline bool Continue() const;
  void End();
  Convertor& ConvertHeader(element::Map::Ptr ele_map);
  Convertor& ConvertJunction(element::Map::Ptr ele_map);
  Convertor& ConvertJunctionAttr(const element::Junction& ele_junction,
                                 core::Junction::Ptr junction);
  Convertor& ConvertRoad(element::Map::Ptr ele_map);
  Convertor& ConvertRoadAttr(const element::Road& ele_road,
                             core::Road::Ptr road);
  Convertor& ConvertSections(const element::Road& ele_road,
                             core::Road::Ptr road);
  Convertor& BuildKDTree();
  void AppendKDTreeSample(const core::Curve::Point& point);
  void CenterLaneSampling(const element::Geometry::Ptrs& geometrys,
                          const element::LaneOffsets& lane_offsets,
                          core::Section::Ptr section, double& road_ds);
  void LaneSampling(const element::Lane& ele_lane, core::Lane::Ptr lane,
                    const core::Curve::Line& refe_line);
  element::Geometry::ConstPtr GetGeometry(
      const element::Geometry::Ptrs& geometrys, double road_ds);
  double GetLaneOffsetValue(const element::LaneOffsets& offsets,
                            double road_ds);
  float step_;
  Status status_;
  common::Param::ConstPtr param_;
  core::Data::Ptr data_;
  core::Curve::Points center_line_pts_;
};

}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CONVERTOR_H_
