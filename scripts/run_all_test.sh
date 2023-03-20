#!/bin/bash

CURRENT_PATH=$(cd $(dirname $0) && pwd)
PEOJECT_PATH="$CURRENT_PATH/.."

export DYLD_LIBRARY_PATH=$PEOJECT_PATH/install/lib:$DYLD_LIBRARY_PATH
export LD_LIBRARY_PATH=$PEOJECT_PATH/install/lib:$LD_LIBRARY_PATH

function test() {
  echo "---------------------------------------------------------------"
  echo "------------------------- run all test ------------------------"
  echo "---------------------------------------------------------------"
  cd $PEOJECT_PATH
  cmake -B build -DBUILD_SHARED_LIBS=ON -DBUILD_OPENDRIVE_ENGINE_TEST=ON -DCMAKE_BUILD_TYPE=Debug .
  cmake --build build -j6
  local bin_path="$CURRENT_PATH/../build/tests"
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

