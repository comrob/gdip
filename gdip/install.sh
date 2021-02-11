#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_INSTALL_PREFIX=.. ../src ..
cd -
cmake --build build --config Release
cmake --install build --config Release
