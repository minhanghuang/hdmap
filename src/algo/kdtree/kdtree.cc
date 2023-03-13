#include "opendrive-engine/algo/kdtree/kdtree.h"

namespace opendrive {
namespace engine {
namespace kdtree {

KDTreeAdaptor::~KDTreeAdaptor() {}

KDTreeAdaptor::KDTreeAdaptor() {}

size_t KDTreeAdaptor::kdtree_get_point_count() const { return matrix_.size(); }

double KDTreeAdaptor::kdtree_get_pt(size_t idx, size_t dim) const {
  return matrix_[idx][dim];
}

void KDTreeAdaptor::Init(const SamplePoints& samples) {
  matrix_.clear();
  ids_.clear();
  matrix_.reserve(samples.size());
  ids_.reserve(samples.size());
  KDTreeNode node(2);
  for (const auto& point : samples) {
    node[0] = point.x();
    node[1] = point.y();
    ids_.emplace_back(point.id());
    matrix_.emplace_back(node);
  }
}
const KDTreeNodes& KDTreeAdaptor::matrix() const { return matrix_; }

const KDTreeIds& KDTreeAdaptor::ids() const { return ids_; }

KDTree::~KDTree() {}

KDTree::KDTree() : index_(nullptr) {}

int KDTree::Search(double x, double y, size_t num_closest,
                   KDTreeResult& result) {
  result.ids.clear();
  result.pts.clear();
  result.dists.clear();
  result.ids.reserve(num_closest);
  result.pts.reserve(num_closest);
  result.dists.reserve(num_closest);
  std::vector<size_t> indices(num_closest);  // 必须设置长度
  std::vector<double> dists(num_closest);    // 必须设置长度
  KDTreeNode query_node{x, y};
  index_->knnSearch(&query_node[0], num_closest, &indices[0], &dists[0]);
  if (indices.size() > adaptor_.matrix().size()) {
    return -1;
  }
  KDTreeNode node(2);
  for (int i = 0; i < indices.size(); i++) {
    node[0] = adaptor_.matrix().at(indices.at(i))[0];
    node[1] = adaptor_.matrix().at(indices.at(i))[1];
    result.pts.emplace_back(node);
    result.ids.emplace_back(adaptor_.ids().at(indices.at(i)));
    result.dists.emplace_back(std::sqrt(dists.at(i)));
  }
  return 0;
}

void KDTree::Init(const SamplePoints& samples, const KDTreeParam& param) {
  std::lock_guard<std::mutex> guard(mutex_);
  nanoflann::KDTreeSingleIndexAdaptorParams adaptor_params;
  adaptor_params.flags = param.flags;
  adaptor_params.leaf_max_size = param.leaf_max_size;
  adaptor_.Init(samples);
  index_.reset(new KDTreeIndex(samples.size(), adaptor_, adaptor_params));
}

}  // namespace kdtree
}  // namespace engine
}  // namespace opendrive
