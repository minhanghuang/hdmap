#include "hdmap/common/utils.h"

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
  if (FILE* f = fopen(file_path.c_str(), "r")) {
    fclose(f);
    return true;
  }
  return false;
}

std::string GetLaneIdById(const std::string& point_id) {
  std::string lane_id;
  auto split_ret = Split(point_id, "_");
  if (5 != split_ret.size()) {
    return lane_id;
  }
  return split_ret[0] + "_" + split_ret[1] + "_" + split_ret[2];
}

}  // namespace common
}  // namespace hdmap
