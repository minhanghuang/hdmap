#ifndef HDMAP_ALGO_KDTREE_H_
#define HDMAP_ALGO_KDTREE_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <nanoflann.hpp>
#include <string>
#include <vector>

#include "hdmap_engine/geometry.h"

namespace hdmap {
namespace kdtree {

using SamplePoint = geometry::Curve::Point;
using SamplePoints = std::vector<SamplePoint>;
using KDTreeNode = std::vector<double>;
using KDTreeNodes = std::vector<KDTreeNode>;
using KDTreeIds = std::vector<std::string>;
using KDTreeIndices = std::vector<size_t>;
using KDTreeDists = std::vector<double>;

struct KDTreeParam {
  KDTreeParam()
      : leaf_max_size(10),
        flags(nanoflann::KDTreeSingleIndexAdaptorFlags::None) {}
  size_t leaf_max_size;
  nanoflann::KDTreeSingleIndexAdaptorFlags flags;
};

struct SearchResult {
  SearchResult() : x(0), y(0), dist(0), id("") {}
  double x;
  double y;
  double dist;  // not sqr
  std::string id;
};
using SearchResults = std::vector<SearchResult>;

class KDTreeAdaptor {
 public:
  ~KDTreeAdaptor();
  KDTreeAdaptor();
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
  size_t kdtree_get_point_count() const;
  double kdtree_get_pt(size_t idx, size_t dim) const;
  void Init(const SamplePoints& samples);
  const KDTreeNodes& matrix() const;
  const KDTreeIds& ids() const;

 private:
  void Clear();
  KDTreeNodes matrix_;
  KDTreeIds ids_;
};

class KDTree {
 public:
  using Ptr = std::shared_ptr<KDTree>;
  using KDTreeIndex = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::metric_L2::template traits<double, KDTreeAdaptor>::distance_t,
      KDTreeAdaptor, 2 /* dimensionality */, size_t /* index type */>;
  ~KDTree();
  KDTree();
  void Init(const SamplePoints& samples,
            const KDTreeParam& param = KDTreeParam());

  template <typename PointType>
  SearchResults Query(const PointType& query_point, size_t num_closest) {
    SearchResults results;
    Search(query_point.x(), query_point.y(), num_closest, results);
    return results;
  }

  template <typename T>
  SearchResults Query(T x, T y, size_t num_closest) {
    SearchResults results;
    Search(static_cast<double>(x), static_cast<double>(y), num_closest,
           results);
    return results;
  }

  template <typename PointType>
  SearchResults QueryByRadius(const PointType& query_point, float radius) {
    SearchResults results;
    RadiusSearch(query_point.x(), query_point.y(), radius, results);
    return results;
  }

  template <typename T>
  SearchResults QueryByRadius(T x, T y, float radius) {
    SearchResults results;
    RadiusSearch(static_cast<double>(x), static_cast<double>(y), radius,
                 results);
    return results;
  }

 private:
  /// Search closest points
  int Search(double x, double y, size_t num_closest, SearchResults& results);

  /// Search points by radius
  int RadiusSearch(double x, double y, float radius, SearchResults& results);

  KDTreeParam param_;

  KDTreeAdaptor adaptor_;

  std::shared_ptr<KDTreeIndex> index_;
};

}  // namespace kdtree
}  // namespace hdmap

#endif  // HDMAP_ALGO_KDTREE_H_
