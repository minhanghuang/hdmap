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
