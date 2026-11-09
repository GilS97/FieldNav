# sensors : modèles de capteurs bruités (roues aujourd'hui, GPS ensuite), aléa seedé.

add_library(fieldnav_sensors STATIC
    wheel_noise.cpp
)

target_include_directories(fieldnav_sensors
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_sensors PUBLIC cxx_std_17)

fieldnav_enable_warnings(fieldnav_sensors)
fieldnav_enable_sanitizers(fieldnav_sensors)

add_library(fieldnav::sensors ALIAS fieldnav_sensors)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
