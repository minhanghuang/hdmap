#!/bin/bash

CURRENT_PATH=$(cd $(dirname $0) && pwd)
PEOJECT_PATH="$CURRENT_PATH/.."

function test() {
  echo "---------------------------------------------------------------"
  echo "------------------------- run tests ------------------------"
  echo "---------------------------------------------------------------"
  cd $PEOJECT_PATH
  mkdir -p build && cd build
  cmake -DBUILD_SHARED_LIBS=ON -DBUILD_OPENDRIVE_ENGINE_TEST=ON -DCMAKE_BUILD_TYPE=Debug ..
  make -j4
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

