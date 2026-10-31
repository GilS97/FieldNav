# sensor_utils : recréé en C++20 : la version de FieldTelemetry est perdue (ADR 0004).
# Header-only : bibliothèque INTERFACE, le standard C++20 est propagé aux cibles qui la lient.

add_library(fieldnav_sensor_utils INTERFACE)

target_include_directories(fieldnav_sensor_utils
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_sensor_utils INTERFACE cxx_std_20)

add_library(fieldnav::sensor_utils ALIAS fieldnav_sensor_utils)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
