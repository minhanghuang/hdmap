#include <iostream>

#include "src/global_data.h"
#include "src/server.h"

int main(int argc, char* argv[]) {
  std::string yaml_file;
  if (2 == argc) {
    yaml_file = argv[1];
  } else {
    std::cerr << "Usage: engine_server_runner xxx.yaml" << std::endl;
    return EXIT_FAILURE;
  }
  auto global_data = opendrive::engine::server::GlobalData::Instance();
  if (global_data->Init(yaml_file)) {
    std::cerr << "yaml init fault." << std::endl;
    return EXIT_FAILURE;
  }

  opendrive::engine::Server server;
  server.Init();
  server.Start();
  return EXIT_SUCCESS;
}
