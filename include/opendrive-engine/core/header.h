#ifndef OPENDRIVE_ENGINE_CORE_HEADER_H_
#define OPENDRIVE_ENGINE_CORE_HEADER_H_

#include <cactus/macros.h>

#include <memory>
#include <string>

namespace opendrive {
namespace engine {
namespace core {

class Header {
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, rev_major);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, rev_minor);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, name);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, version);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, date);
  CACTUS_REGISTER_MEMBER_COMPLEX_TYPE(std::string, vendor);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, north, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, south, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, west, 0);
  CACTUS_REGISTER_MEMBER_BASIC_TYPE(double, east, 0);

 public:
  using Ptr = std::shared_ptr<Header>;
  using ConstPtr = std::shared_ptr<Header const>;
  Header() {}
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_HEADER_H_
