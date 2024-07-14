#include "hdmap_engine/core/kdtree/kdtree.h"

namespace hdmap {
namespace kdtree {

KDTreeAdaptor::~KDTreeAdaptor() {}

KDTreeAdaptor::KDTreeAdaptor() {}

void KDTreeAdaptor::Clear() {
  KDTreeNodes().swap(matrix_);
  KDTreeIds().swap(ids_);
}

size_t KDTreeAdaptor::kdtree_get_point_count() const { return matrix_.size(); }

double KDTreeAdaptor::kdtree_get_pt(size_t idx, size_t dim) const {
  return matrix_[idx][dim];
}

void KDTreeAdaptor::Init(const SamplePoints& samples) {
  Clear();
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
                   SearchResults& results) {
  results.clear();
  KDTreeIndices indices(num_closest);  // 必须设置长度
  KDTreeDists dists(num_closest);      // 必须设置长度
  KDTreeNode query_node{x, y};
  index_->knnSearch(&query_node[0], num_closest, &indices[0], &dists[0]);
  if (indices.size() > adaptor_.matrix().size()) {
    return -1;
  }
  SearchResult result;
  for (int i = 0; i < indices.size(); i++) {
    result.x = adaptor_.matrix().at(indices.at(i))[0];
    result.y = adaptor_.matrix().at(indices.at(i))[1];
    result.id = adaptor_.ids().at(indices.at(i));
    result.dist = std::sqrt(dists.at(i));
    results.emplace_back(result);
  }
  return 0;
}

int KDTree::RadiusSearch(double x, double y, float radius,
                         SearchResults& results) {
  results.clear();
  std::vector<nanoflann::ResultItem<size_t, double>> search_results;
  KDTreeNode query_node{x, y};
  // If L2 norms are used, radius distances are actually squared distances.
  index_->radiusSearch(&query_node[0], std::pow(radius, 2), search_results);
  SearchResult result;
  for (const auto& search_result : search_results) {
    result.x = adaptor_.matrix().at(search_result.first)[0];
    result.y = adaptor_.matrix().at(search_result.first)[1];
    result.id = adaptor_.ids().at(search_result.first);
    result.dist = std::sqrt(search_result.second);
    results.emplace_back(result);
  }
  return 0;
}

void KDTree::Init(const SamplePoints& samples, const KDTreeParam& param) {
  nanoflann::KDTreeSingleIndexAdaptorParams adaptor_params;
  adaptor_params.flags = param.flags;
  adaptor_params.leaf_max_size = param.leaf_max_size;
  adaptor_.Init(samples);
  index_.reset(new KDTreeIndex(samples.size(), adaptor_, adaptor_params));
}

}  // namespace kdtree
}  // namespace hdmap
