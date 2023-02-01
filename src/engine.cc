#include "opendrive-engine/engine.h"

namespace opendrive {
namespace engine {

Engine::Engine() : impl_(std::make_shared<EngineImpl>()) {}

int Engine::Init(const common::Param& param) { return impl_->Init(param); }

}  // namespace engine
}  // namespace opendrive
