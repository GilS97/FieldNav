# estimation : fusion complémentaire odométrie + GPS.

add_library(fieldnav_estimation STATIC
    complementary.cpp
)

target_include_directories(fieldnav_estimation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_estimation PUBLIC cxx_std_17)
target_link_libraries(fieldnav_estimation PUBLIC fieldnav::odometry fieldnav::sensors)

fieldnav_enable_warnings(fieldnav_estimation)
fieldnav_enable_sanitizers(fieldnav_estimation)

add_library(fieldnav::estimation ALIAS fieldnav_estimation)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
