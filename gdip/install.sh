#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_INSTALL_PREFIX=.. ../src ..
cd -

if [[ "$OSTYPE" == "win32" ]]; then
    cmake --build build --config Release
    cmake --install build --config Release
else
    cmake --build build
    cmake --install build
fi


