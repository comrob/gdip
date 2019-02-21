cmake_minimum_required(VERSION 2.8)
project(opendubins_wrappers)

include_directories("../core/src")
include_directories("src")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        ../core/src/opendubins/arc.cpp
        ../core/src/opendubins/circle.cpp
        ../core/src/opendubins/dubins.cpp
        ../core/src/opendubins/dubinsmaneuver.cpp
        ../core/src/opendubins/dip.cpp
        ../core/src/opendubins/gdip.cpp
        ../core/src/opendubins/line.cpp
        ../core/src/opendubins/path.cpp
        ../core/src/opendubins/state.cpp
        ../core/src/opendubins/dubinspath.cpp
        ../core/src/opendubins/angleinterval.cpp

        src/gdipPythonAPI.cpp
        src/gdipJuliaAPI.cpp
)

add_library(GDIP SHARED ${SOURCE_FILES})

install(TARGETS GDIP DESTINATION lib)
