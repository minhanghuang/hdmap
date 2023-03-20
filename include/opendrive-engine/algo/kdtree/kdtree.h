#ifndef OPENDRIVE_ENGINE_ALGO_KDTREE_H_
#define OPENDRIVE_ENGINE_ALGO_KDTREE_H_

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <nanoflann.hpp>
#include <string>
#include <vector>

#include "cactus/rw_lock.h"
#include "opendrive-engine/core/id.h"
#include "opendrive-engine/core/lane.h"

namespace opendrive {
namespace engine {
namespace kdtree {

typedef core::Curve::Point SamplePoint;
typedef std::vector<SamplePoint> SamplePoints;
typedef std::vector<double> KDTreeNode;
typedef std::vector<KDTreeNode> KDTreeNodes;
typedef std::vector<core::Id> KDTreeIds;
typedef std::vector<size_t> KDTreeIndices;
typedef std::vector<double> KDTreeDists;

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
  core::Id id;
};
typedef std::vector<SearchResult> SearchResults;

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
  typedef std::shared_ptr<KDTree> Ptr;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::metric_L2::template traits<double, KDTreeAdaptor>::distance_t,
      KDTreeAdaptor, 2 /* dimensionality */, size_t /* index type */>
      KDTreeIndex;
  ~KDTree();
  KDTree();
  void Init(const SamplePoints& samples,
            const KDTreeParam& param = KDTreeParam());

  template <typename PointType>
  SearchResults Query(const PointType& query_point, size_t num_closest) {
    cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
    SearchResults result;
    Search(query_point.x(), query_point.y(), num_closest, result);
    return result;
  }

  template <typename T>
  SearchResults Query(T x, T y, size_t num_closest) {
    cactus::ReadLockGuard<cactus::AtomicRWLock> guard(rw_lock_);
    SearchResults result;
    Search(static_cast<double>(x), static_cast<double>(y), num_closest, result);
    return result;
  }

 private:
  int Search(double x, double y, size_t num_closest, SearchResults& result);
  cactus::AtomicRWLock rw_lock_;  // read and write lock
  KDTreeParam param_;
  KDTreeAdaptor adaptor_;
  std::shared_ptr<KDTreeIndex> index_;
};

}  // namespace kdtree
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_ALGO_KDTREE_H_
