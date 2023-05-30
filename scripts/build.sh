#!/bin/bash

function main() {
  echo "---------------------------------------------------------------"
  echo "--------------------- opendrive-engine build ------------------"
  echo "---------------------------------------------------------------"
  cmake -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Debug .
  cmake --build build -j6
}

main "$@"
