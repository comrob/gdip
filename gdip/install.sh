#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_INSTALL_PREFIX=.. ../src ..
cd -

if [[ "$OSTYPE" == "win32" ]]; then
    echo "Compiling on Windows"
    echo "cmake --build build --config Release"
    cmake --build build --config Release
    echo "cmake --install build --config Release"
    cmake --install build --config Release
else
    echo "Compiling Linux/OSX"
    echo "cmake --build build"
    cmake --build build
    echo "cmake --install build"
    cmake --install build
fi


