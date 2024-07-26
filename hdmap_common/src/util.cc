#include "hdmap_common/util.h"

namespace hdmap {
namespace common {

std::vector<std::string> Split(const std::string& input,
                               const std::string& delimiter) {
  std::vector<std::string> result;
  boost::split(result, input, boost::is_any_of(delimiter),
               boost::token_compress_on);
  return result;
}

bool ApproximatelyEqual(double a, double b, double tolerance) {
  // Calculate the relative difference using Boost's function
  // and compare it to the specified tolerance
  return boost::math::relative_difference(a, b) <= tolerance;
}

std::string ShortenPath(const std::string& path, size_t max_length) {
  if (path.length() <= max_length) {
    return path;
  }
  size_t keep = (max_length - 3) / 2;
  return path.substr(0, keep) + "..." + path.substr(path.length() - keep);
}

std::string GenerateUuid() {
  boost::uuids::random_generator gen;
  return boost::uuids::to_string(gen());
}

std::string GetLaneIdByPointId(const std::string& point_id) {
  auto split_ret = Split(point_id, "_");
  if (5 != split_ret.size()) {
    return std::string("");
  }
  if (!std::none_of(split_ret.begin(), split_ret.end(),
                    [](const std::string& s) { return s.empty(); })) {
    return std::string("");
  }
  return std::string(split_ret[0] + "_" + split_ret[1] + "_" + split_ret[2]);
}

}  // namespace common
}  // namespace hdmap
