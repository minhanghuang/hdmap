#ifndef OPENDRIVE_ENGINE_CORE_JUNCTION_H_
#define OPENDRIVE_ENGINE_CORE_JUNCTION_H_

#include <memory>

#include "geometry.h"
#include "id.h"

namespace opendrive {
namespace engine {
namespace core {

class Junction {
 public:
  Junction() = default;
  typedef std::shared_ptr<Junction> Ptr;
  typedef std::shared_ptr<Junction const> ConstPtr;
  void set_id(const Id& s) { id_ = s; }
  void set_name(const std::string& s) { name_ = s; }
  void set_type(JunctionType t) { type_ = t; }
  const Id& id() { return id_; }
  const std::string& name() { return name_; }
  JunctionType type() { return type_; }

 private:
  Id id_;
  std::string name_;
  JunctionType type_ = JunctionType::DEFAULT;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_JUNCTION_H_
