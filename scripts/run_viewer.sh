#!/bin/bash

CURRENT_PATH=$(cd $(dirname $0) && pwd)
PEOJECT_PATH="$CURRENT_PATH/.."

export DYLD_LIBRARY_PATH=$PEOJECT_PATH/install/lib:$DYLD_LIBRARY_PATH
export LD_LIBRARY_PATH=$PEOJECT_PATH/install/lib:$LD_LIBRARY_PATH

function main() {
  echo "---------------------------------------------------------------"
  echo "--------------------- opendrive-engine-viewer run -------------"
  echo "---------------------------------------------------------------"
  cmake -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_OPENDRIVE_ENGINE_VIEWER=ON .
  cmake --build build -j6
  build/viewer/backend/engine_server_runner build/conf/engine_server.yaml
}

main "$@"
