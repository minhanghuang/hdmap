#include "opendrive-engine/engine.h"

namespace opendrive {
namespace engine {

Engine::~Engine() {}
Engine::Engine(const common::Param& param)
    : impl_(std::make_shared<EngineImpl>(param)) {
  this->Init();
}

Engine::Engine(common::Param::Ptr param)
    : impl_(std::make_shared<EngineImpl>(param)) {
  this->Init();
}

void Engine::Init() {}

}  // namespace engine
}  // namespace opendrive
