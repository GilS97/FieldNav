# Tests de sensor_utils (GoogleTest, fourni par le CMakeLists racine via FetchContent).
# Un fichier de test par en-tête de include/fieldnav/sensor_utils/

add_executable(test_sensor_utils
    test_stats.cpp
)
target_link_libraries(test_sensor_utils PRIVATE fieldnav::sensor_utils GTest::gtest_main)
fieldnav_enable_warnings(test_sensor_utils)
fieldnav_enable_sanitizers(test_sensor_utils)

add_test(NAME sensor_utils.unit COMMAND test_sensor_utils)

# Critère de fini de l'import : plus aucune construction postérieure à C++20 dans le module.
add_test(NAME sensor_utils.no_cxx23
    COMMAND ${CMAKE_COMMAND}
        -DSCAN_DIR=${PROJECT_SOURCE_DIR}/libs/sensor_utils
        -P ${PROJECT_SOURCE_DIR}/cmake/CheckNoCxx23.cmake
)
