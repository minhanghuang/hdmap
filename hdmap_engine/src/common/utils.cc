#include "hdmap_engine/common/utils.h"

namespace hdmap {
namespace common {

Tokens Split(const std::string& tokenstring, const std::string& delimiter) {
  Tokens ret;
  if (tokenstring.empty()) return ret;
  size_t pos = 0;
  size_t index = 0;
  std::string token;
  while ((pos = tokenstring.find(delimiter, index)) != std::string::npos) {
    token = tokenstring.substr(index, pos - index);
    ret.emplace_back(token);
    index = pos + delimiter.length();
  }
  token = tokenstring.substr(index);
  ret.emplace_back(token);
  return ret;
}

bool FileExists(const std::string& file_path) {
  return boost::filesystem::exists(file_path);
}

std::string GetLaneIdById(const std::string& point_id) {
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
