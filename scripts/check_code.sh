#!/bin/bash

current_path=$(cd $(dirname $0) && pwd)

function run() {
  echo "---------------------------------------------------------------"
  echo "------------------------- run cppcheck ------------------------"
  echo "---------------------------------------------------------------"
  cppcheck \
    --enable=all \
    --error-exitcode=1 \
    --std=c++14  \
    --inconclusive \
    --language=c++ \
    --force \
    ${current_path}/../src/
}

function main () {
  run 
  return
}

main "$@"
