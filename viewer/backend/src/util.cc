#include "util.h"

#include <algorithm>

namespace opendrive {
namespace engine {
namespace server {

bool ConvertLineToPts(const core::Curve& line, Json& line_json) {
  line_json.clear();
  for (int i = 0; i < line.pts().size(); i++) {
    line_json[i][0] = line.pts().at(i).x();
    line_json[i][1] = line.pts().at(i).y();
  }
  return true;
}

}  // namespace server
}  // namespace engine
}  // namespace opendrive
