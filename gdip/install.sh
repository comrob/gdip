#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_INSTALL_PREFIX=.. ../src ..
make install -j4 || nmake install -j4
cd -
