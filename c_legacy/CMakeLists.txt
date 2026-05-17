# Phase 1: C matrix library (educational).

add_library(fieldnav_c_legacy STATIC
    src/matrix.c
)

target_include_directories(fieldnav_c_legacy
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_c_legacy PUBLIC c_std_11)

fieldnav_enable_warnings(fieldnav_c_legacy)
fieldnav_enable_sanitizers(fieldnav_c_legacy)

add_library(fieldnav::c_legacy ALIAS fieldnav_c_legacy)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
