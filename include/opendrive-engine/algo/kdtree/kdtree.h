#ifndef OPENDRIVE_ENGINE_ALGO_KDTREE_H_
#define OPENDRIVE_ENGINE_ALGO_KDTREE_H_

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <nanoflann.hpp>
#include <string>
#include <vector>

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

struct KDTreeResult {
  KDTreeNodes pts;
  KDTreeIds ids;
  KDTreeDists dists;  // not sqr
};

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
  KDTreeNodes matrix_;
  KDTreeIds ids_;
};

class KDTree {
 public:
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::metric_L2::template traits<double, KDTreeAdaptor>::distance_t,
      KDTreeAdaptor, 2 /* dimensionality */, size_t /* index type */>
      KDTreeIndex;
  ~KDTree();
  KDTree();
  void Init(const SamplePoints& samples,
            const KDTreeParam& param = KDTreeParam());

  template <typename PointType>
  KDTreeResult Query(const PointType& query_point, size_t num_closest) {
    std::lock_guard<std::mutex> guard(mutex_);
    KDTreeResult result;
    Search(query_point.x(), query_point.y(), num_closest, result);
    return result;
  }

  template <typename T>
  KDTreeResult Query(T x, T y, size_t num_closest) {
    std::lock_guard<std::mutex> guard(mutex_);
    KDTreeResult result;
    Search(static_cast<double>(x), static_cast<double>(y), num_closest, result);
    return result;
  }

 private:
  int Search(double x, double y, size_t num_closest, KDTreeResult& result);
  std::mutex mutex_;
  KDTreeParam param_;
  KDTreeAdaptor adaptor_;
  std::shared_ptr<KDTreeIndex> index_;
};

}  // namespace kdtree
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_ALGO_KDTREE_H_
