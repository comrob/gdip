#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=.. ../src ..
cd -
cmake --build build
