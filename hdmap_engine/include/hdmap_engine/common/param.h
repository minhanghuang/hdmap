#ifndef HDMAP_COMMON_PARAM_H_
#define HDMAP_COMMON_PARAM_H_

#include <memory>
#include <string>

#include "hdmap_engine/common/macros.h"

namespace hdmap {

class Param {
  ADD_MEMBER_COMPLEX_TYPE(std::string, file_path)
  ADD_MEMBER_BASIC_TYPE(float, step, 0.5)

 public:
  using Ptr = std::shared_ptr<Param>;
  using ConstPtr = std::shared_ptr<Param const>;
};

}  // namespace hdmap

#endif  // HDMAP_COMMON_PARAM_H_
