#!/bin/bash

export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/opt/xodr/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/xodr/lib

function main() {
  echo "---------------------------------------------------------------"
  echo "--------------------- opendrive-engine-viewer run -------------"
  echo "---------------------------------------------------------------"
  cmake -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_OPENDRIVE_ENGINE_VIEWER=ON .
  cmake --build build -j6
  build/viewer/backend/engine_server_runner build/conf/engine_server.yaml
}

main "$@"
