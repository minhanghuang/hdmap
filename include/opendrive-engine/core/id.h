#ifndef OPENDRIVE_ENGINE_CORE_ID_H_
#define OPENDRIVE_ENGINE_CORE_ID_H_

#include <string>
#include <unordered_set>

namespace opendrive {
namespace engine {
namespace core {

using Id = std::string;
using Ids = std::unordered_set<Id>;
using Path = std::vector<Id>;

}  // namespace core
}  // namespace engine
}  // namespace opendrive

#endif  // OPENDRIVE_ENGINE_CORE_ID_H_
