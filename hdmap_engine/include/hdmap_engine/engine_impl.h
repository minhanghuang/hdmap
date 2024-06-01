#ifndef HDMAP_ENGINE_IMPL_H_
#define HDMAP_ENGINE_IMPL_H_

#include <memory>
#include <string>

#include "hdmap_engine/adapter/opendrive/opendrive_convertor.h"
#include "hdmap_engine/adapter/opendrive/opendrive_parser.h"
#include "hdmap_engine/adapter/parser.h"
#include "hdmap_engine/algo/kdtree/kdtree.h"
#include "hdmap_engine/algo/routing/routing.h"
#include "hdmap_engine/common/log.h"
#include "hdmap_engine/common/param.h"
#include "hdmap_engine/common/status.h"
#include "hdmap_engine/common/utils.h"

namespace hdmap {

class EngineImpl {
 public:
  using Ptr = std::shared_ptr<EngineImpl>;
  EngineImpl();

  /**
   * @brief init
   *
   * @param param
   * @return
   */
  bool Init(const Param& param);

  /**
   * @brief Get lane point by point id
   *
   * @param point_id
   * @param out_point
   * @return
   */
  bool GetPointById(const std::string& point_id,
                    geometry::Curve::Point& out_point);

  /**
   * @brief Get lane ptr by lane id
   *
   * @param id
   */
  geometry::Lane::ConstPtr GetLaneById(const std::string& id) const;

  /**
   * @brief Get section ptr by section id
   *
   * @param id
   */
  geometry::Section::ConstPtr GetSectionById(const std::string& id) const;

  /**
   * @brief Get road ptr by road id
   *
   * @param id
   */
  geometry::Road::ConstPtr GetRoadById(const std::string& id) const;

  /**
   * @brief Get all lane ptr of the map
   */
  geometry::Lane::ConstPtrs GetLanes() const;

  /**
   * @brief Get all section ptr of the map
   */
  geometry::Section::ConstPtrs GetSections() const;

  /**
   * @brief Get all road ptr of the map
   */
  geometry::Road::ConstPtrs GetRoads() const;

  /**
   * @brief Get header ptr of the map
   */
  geometry::Header::ConstPtr GetHeader() const;

  /**
   * @brief Get data of the nearest point through target point
   *
   * @param x: target point x
   * @param y: target point y
   * @param num_closest: nearest point number
   */
  kdtree::SearchResults GetNearestPoints(double x, double y,
                                         size_t num_closest);
  /**
   * @brief Get data from the lane where the nearest point to the target point
   * is located.
   *
   * @param x: target point x
   * @param y: target point y
   * @param num_closest: nearest point number
   */
  geometry::Lane::ConstPtrs GetNearestLanes(double x, double y,
                                            size_t num_closest);

 private:
  bool Checkin();

  /**
   * @brief engine param
   */
  Param::Ptr param_;

  /**
   * @brief engine status singleton
   */
  Status status_;

  /**
   * @brief map data
   */
  geometry::Map::Ptr map_;

  /**
   * @brief knn search
   */
  kdtree::KDTree::Ptr kdtree_;
};

}  // namespace hdmap

#endif  // HDMAP_ENGINE_IMPL_H_
