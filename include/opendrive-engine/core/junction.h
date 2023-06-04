#ifndef OPENDRIVE_ENGINE_CORE_JUNCTION_H_
#define OPENDRIVE_ENGINE_CORE_JUNCTION_H_

#include <memory>

#include "id.h"
#include "opendrive-engine/geometry/geometry.h"

namespace opendrive {
namespace engine {
namespace core {

class Junction {
 public:
  typedef std::shared_ptr<Junction> Ptr;
  typedef std::shared_ptr<Junction const> ConstPtr;
  Junction() : id_(""), name_(""), type_(JunctionType::kDefault) {}
  void set_id(const Id& s) { id_ = s; }
  void set_name(const std::string& s) { name_ = s; }
  void set_type(JunctionType t) { type_ = t; }
  Id& mutable_id() { return id_; }
  std::string& mutable_name() { return name_; }
  JunctionType& mutable_type() { return type_; }
  const Id& id() const { return id_; }
  const std::string& name() const { return name_; }
  JunctionType type() const { return type_; }

 private:
  Id id_;
  std::string name_;
  JunctionType type_;
};

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_JUNCTION_H_
