#!/bin/bash

current_path=$(cd $(dirname $0) && pwd)

export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/opt/base/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/base/lib

function test() {
  echo "---------------------------------------------------------------"
  echo "------------------------- run all test ------------------------"
  echo "---------------------------------------------------------------"
  cmake -B build -DBUILD_SHARED_LIBS=ON -DBUILD_OPENDRIVE_ENGINE_TEST=ON -DCMAKE_BUILD_TYPE=Debug .
  cmake --build build -j6
  local bin_path="$current_path/../build/tests"
  for file in $bin_path/*
  do
    case "$file" in
      *test )
        echo "---run: $file"
        $file
        ;;
      * );;
    esac
  done
}

function main () {
  test
  return
}

main "$@"

