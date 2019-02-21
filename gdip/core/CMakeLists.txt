cmake_minimum_required(VERSION 2.8)
project(opendubins_core)

include_directories("src")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES ${DUBINS_SRC}
        src/opendubins/arc.cpp
        src/opendubins/circle.cpp
        src/opendubins/dubins.cpp
        src/opendubins/dubinsmaneuver.cpp
        src/opendubins/dip.cpp
        src/opendubins/gdip.cpp
        src/opendubins/line.cpp
        src/opendubins/path.cpp
        src/opendubins/state.cpp
        src/opendubins/dubinspath.cpp
        src/opendubins/angleinterval.cpp

        src/opendubins/stateatdistance.h
        src/opendubins/vector.h
        src/opendubins/point.h
        src/opendubins/dmath.h
        src/opendubins/dtype.h)

add_library(opendubins_core STATIC ${SOURCE_FILES})

if (GTEST_FOUND)

    include_directories("${GTEST_INCLUDE_DIRS}")

    add_executable(dubinstest
            test/dubinstest.cpp
            test/diptest.cpp
            test/gdiptest.cpp
            test/circletest.cpp
            test/angleintervaltest.cpp
            test/speedtest.cpp
            )

    target_link_libraries(dubinstest opendubins_core)
    target_link_libraries(dubinstest ${GTEST_BOTH_LIBRARIES} pthread )

    add_test(core_tests dubinstest)

    install(TARGETS dubinstest DESTINATION bin)

endif ()

install(DIRECTORY src/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(TARGETS opendubins_core DESTINATION lib)
