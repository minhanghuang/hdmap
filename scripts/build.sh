#!/bin/bash

CURRENT_PATH=$(cd $(dirname $0) && pwd)
PEOJECT_PATH="$CURRENT_PATH/.."

export DYLD_LIBRARY_PATH=$PEOJECT_PATH/install/lib:$DYLD_LIBRARY_PATH
export LD_LIBRARY_PATH=$PEOJECT_PATH/install/lib:$LD_LIBRARY_PATH

function main() {
  echo "---------------------------------------------------------------"
  echo "--------------------- opendrive-engine build ------------------"
  echo "---------------------------------------------------------------"
  cmake -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug .
  cmake --build build -j6
}

main "$@"
