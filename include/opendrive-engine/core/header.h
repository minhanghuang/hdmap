#ifndef OPENDRIVE_ENGINE_CORE_HEADER_H_
#define OPENDRIVE_ENGINE_CORE_HEADER_H_

#include <memory>
#include <string>

namespace opendrive {
namespace engine {
namespace core {

typedef struct Header HeaderTypedef;
struct Header {
  typedef std::shared_ptr<HeaderTypedef> Ptr;
  typedef std::shared_ptr<HeaderTypedef const> ConstPtr;
  std::string rev_major;
  std::string rev_minor;
  std::string name;
  std::string version;
  std::string date;
  double north = 0.;
  double south = 0.;
  double west = 0.;
  double east = 0.;
  std::string vendor;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_HEADER_H_
