# mission : la boucle complète sim → capteurs → estimée → pure pursuit → roues, et la télémétrie.

add_library(fieldnav_mission STATIC
    mission.cpp
)

target_include_directories(fieldnav_mission
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_mission PUBLIC cxx_std_17)
target_link_libraries(fieldnav_mission
    PUBLIC  fieldnav::sensors
    PRIVATE fieldnav::sim fieldnav::odometry fieldnav::estimation fieldnav::path
            fieldnav::control fieldnav::telemetry
)

fieldnav_enable_warnings(fieldnav_mission)
fieldnav_enable_sanitizers(fieldnav_mission)

add_library(fieldnav::mission ALIAS fieldnav_mission)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
