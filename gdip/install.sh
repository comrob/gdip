#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_INSTALL_PREFIX=.. ../src ..
cmake --build .
cd -
