#ifndef HDMAP_ENGINE_H_
#define HDMAP_ENGINE_H_

#include <memory>
#include <string>

#include "hdmap/common/param.h"
#include "hdmap/common/status.h"
#include "hdmap/engine_impl.h"
#include "hdmap/geometry.h"

namespace hdmap {

class Engine {
 public:
  using Ptr = std::shared_ptr<Engine>;
  using ConstPtr = std::shared_ptr<Engine const>;

  ~Engine() = default;

  Engine();

  /**
   * @brief init
   *
   * @param param
   * @return
   */
  bool Init(const Param& param);

  /**
   * @brief engine status
   */
  Status::ConstPtr status() const;

  bool HotUpdate(const Param& param);

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
  geometry::Lane::ConstPtr GetLaneById(const std::string& id);

  /**
   * @brief Get section ptr by section id
   *
   * @param id
   */
  geometry::Section::ConstPtr GetSectionById(const std::string& id);

  /**
   * @brief Get road ptr by road id
   *
   * @param id
   */
  geometry::Road::ConstPtr GetRoadById(const std::string& id);

  /**
   * @brief Get all lane ptr of the map
   */
  geometry::Lane::ConstPtrs GetLanes();

  /**
   * @brief Get all section ptr of the map
   */
  geometry::Section::ConstPtrs GetSections();

  /**
   * @brief Get all road ptr of the map
   */
  geometry::Road::ConstPtrs GetRoads();

  /**
   * @brief Get header ptr of the map
   */
  geometry::Header::ConstPtr GetHeader();

  /**
   * @brief Get data of the nearest point through target point
   *
   * @tparam T:
   * @param x: target point x
   * @param y: target point y
   * @param num_closest: nearest point number
   * @return
   */
  template <typename T>
  kdtree::SearchResults GetNearestPoints(T x, T y, size_t num_closest) {
    return impl_->GetNearestPoints(static_cast<double>(x),
                                   static_cast<double>(y), num_closest);
  }

  /**
   * @brief Get data of the nearest point through target point
   *
   * @tparam T
   * @param query_point: target point
   * @param num_closest: nearest point number
   * @return
   */
  template <typename T>
  kdtree::SearchResults GetNearestPoints(const T& query_point,
                                         size_t num_closest) {
    return impl_->GetNearestPoints(query_point.x(), query_point.y(),
                                   num_closest);
  }

  /**
   * @brief Get data from the lane where the nearest point to the target point
   * is located.
   *
   * @tparam T
   * @param x: target point x
   * @param y: target point y
   * @param num_closest: nearest point number
   * @return
   */
  template <typename T>
  geometry::Lane::ConstPtrs GetNearestLanes(T x, T y, size_t num_closest) {
    return impl_->GetNearestLanes(static_cast<double>(x),
                                  static_cast<double>(y), num_closest);
  }

  /**
   * @brief Get data from the lane where the nearest point to the target point
   * is located.
   *
   * @tparam T
   * @param query_point: target point
   * @param num_closest: nearest point number
   * @return
   */
  template <typename T>
  geometry::Lane::ConstPtrs GetNearestLanes(const T& query_point,
                                            size_t num_closest) {
    return impl_->GetNearestLanes(query_point.x(), query_point.y(),
                                  num_closest);
  }

 private:
  /**
   * @brief engine status
   */
  Status::Ptr status_;

  /**
   * @brief engine imp ptr
   */
  EngineImpl::Ptr impl_;
};

}  // namespace hdmap

#endif  // HDMAP_ENGINE_H_
