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

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

set(GTEST_INCLUDE_DIRS "${gtest_SOURCE_DIR}/include")
message("GTEST_INCLUDE_DIRS = ${GTEST_INCLUDE_DIRS}")

set(GTEST_BOTH_LIBRARIES "gtest_main")
message("GTEST_BOTH_LIBRARIES = ${GTEST_BOTH_LIBRARIES}")

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${GTEST_INCLUDE_DIRS}")
endif()

set(GTEST_FOUND true)

if(GTEST_FOUND)
    MESSAGE("GTEST  OK")
else()
    MESSAGE("GTEST  not found")
endif()

# DIRECTORIES
add_subdirectory(core)
add_subdirectory(examples)
add_subdirectory(wrappers)

