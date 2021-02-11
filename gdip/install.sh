#!/usr/bin/env bash

mkdir -p build
cd build && cmake -D CMAKE_INSTALL_PREFIX=.. ../src ..
cd -

env

UNAME=$(uname)

if [[ "$UNAME" == CYGWIN* || "$UNAME" == MINGW* ]] ; then
	echo "Compiling on Windows"
    echo "cmake --build build --config Release"
    cmake --build build --config Release
    echo "cmake --install build --config Release"
    cmake --install build --config Release
else
    echo "Compiling Linux/OSX"
    echo "make install -j4"
    make -C build install -j4
fi
