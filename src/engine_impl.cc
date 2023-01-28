#include "opendrive-engine/engine_impl.h"

namespace opendrive {
namespace engine {

EngineImpl::EngineImpl(const common::Param& param)
    : param_(std::make_shared<common::Param>(param)) {}
EngineImpl::EngineImpl(common::Param::ConstPtr param) : param_(param) {}

void EngineImpl::Clear() {}

}  // namespace engine
}  // namespace opendrive
