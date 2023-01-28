#!/bin/bash

export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/opt/xodr/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/xodr/lib

function main() {
  echo "---------------------------------------------------------------"
  echo "--------------------- opendrive-engine build ------------------"
  echo "---------------------------------------------------------------"
  cmake -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug .
  cmake --build build -j6
}

main "$@"
