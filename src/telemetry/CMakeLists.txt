# telemetry : écriture de la télémétrie (docs/telemetry_format.md), CSV et JSON Lines.

add_library(fieldnav_telemetry STATIC
    telemetry_writer.cpp
)

target_include_directories(fieldnav_telemetry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_telemetry PUBLIC cxx_std_17)

fieldnav_enable_warnings(fieldnav_telemetry)
fieldnav_enable_sanitizers(fieldnav_telemetry)

add_library(fieldnav::telemetry ALIAS fieldnav_telemetry)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
