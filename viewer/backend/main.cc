#include <iostream>

#include "src/server.h"

int main(int argc, char* argv[]) {
  std::cout << "opendrive engine viewer." << std::endl;
  std::string yaml_file;
  if (2 == argc) {
    yaml_file = argv[1];
  } else {
    exit(1);
  }
  opendrive::engine::server::Server server;
  server.Init(yaml_file);
  server.Start();
  return 0;
}
