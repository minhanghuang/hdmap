#!/bin/bash

function main() {
  echo "---------------------------------------------------------------"
  echo "--------------------- opendrive-engine-viewer run -------------"
  echo "---------------------------------------------------------------"
  mkdir -p build && cd build
  cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_OPENDRIVE_ENGINE_VIEWER=ON ..
  make -j4
  viewer/backend/engine_server_runner conf/engine_server.yaml
}

main "$@"
