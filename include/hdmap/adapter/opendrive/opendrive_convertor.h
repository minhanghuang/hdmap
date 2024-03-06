#ifndef HDMAP_ADAPTER_OPRNDRIVE_PARSER_H_
#define HDMAP_ADAPTER_OPRNDRIVE_PARSER_H_

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include "hdmap/adapter/convertor.h"
#include "hdmap/adapter/opendrive/element.h"
#include "hdmap/algo/kdtree/kdtree.h"
#include "hdmap/common/param.h"
#include "hdmap/common/status.h"
#include "hdmap/common/utils.h"
#include "hdmap/geometry.h"

namespace hdmap {

class OpenDriveConvertor : public ConvertProcessor {
 public:
  virtual bool process(std::shared_ptr<PipelineData>) override;

 private:
  OpenDriveConvertor& CheckIn(std::shared_ptr<PipelineData>);

  OpenDriveConvertor& Convert();

  bool Finish();

  OpenDriveConvertor& ConvertHeader(opendrive::element::Map::Ptr ele_map);

  OpenDriveConvertor& ConvertRoad(opendrive::element::Map::Ptr ele_map);

  OpenDriveConvertor& ConvertRoadAttr(const opendrive::element::Road& ele_road,
                                      geometry::Road::Ptr road);

  OpenDriveConvertor& ConvertSections(const opendrive::element::Road& ele_road,
                                      geometry::Road::Ptr road);

  OpenDriveConvertor& BuildKDTree();

  void AppendKDTreeSample(const geometry::Curve::Point& point);

  void CenterLaneSampling(const opendrive::element::Geometry::Ptrs& geometrys,
                          const opendrive::element::LaneOffsets& lane_offsets,
                          geometry::Section::Ptr section, double& road_ds);

  void LaneSampling(const opendrive::element::Lane& ele_lane,
                    geometry::Lane::Ptr lane,
                    const geometry::Curve::Line* refe_line);

  opendrive::element::Geometry::ConstPtr GetGeometry(
      const opendrive::element::Geometry::Ptrs& geometrys, double road_ds);

  double GetLaneOffsetValue(const opendrive::element::LaneOffsets& offsets,
                            double road_ds);

  float step_ = 0.5;

  geometry::Curve::Points center_line_pts_;
};

}  // namespace hdmap

#endif  // HDMAP_ADAPTER_OPRNDRIVE_PARSER_H_
