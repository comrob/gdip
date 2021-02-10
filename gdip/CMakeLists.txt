cmake_minimum_required(VERSION 2.8)
project(opendubins)

# Debug / Release
if(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE("Compiling in the Debug mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
else()
    MESSAGE("Compiling in the Release mode")
    if (WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
    endif()
    set(CMAKE_BUILD_TYPE Release)
endif()

if (WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

# For debug and profilling:
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")

MESSAGE("CMAKE_CXX_COMPILER_ID  ${CMAKE_CXX_COMPILER_ID}")
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    MESSAGE("Disable error messsages for unused-command-line-argument (Clang only)")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-command-line-argument -Qunused-arguments")
endif()
MESSAGE("CMAKE_C_COMPILER_ID  ${CMAKE_C_COMPILER_ID}")
if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
    MESSAGE("Disable error messsages for unused-command-line-argument (Clang only)")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error=unused-command-line-argument -Qunused-arguments")
endif()

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

enable_testing()
find_package(GTest)

if(GTEST_FOUND)
    MESSAGE("GTEST  OK")
    include_directories(${GTEST_INCLUDE_DIRS})
else()
    MESSAGE("GTEST  not found")
endif()

# DIRECTORIES
add_subdirectory(core)
add_subdirectory(examples)
add_subdirectory(wrappers)

