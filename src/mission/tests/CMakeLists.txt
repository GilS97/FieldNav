# Tests d'intégration de la mission complète (GoogleTest).

add_executable(test_mission
    test_mission.cpp
)
target_link_libraries(test_mission PRIVATE fieldnav::mission GTest::gtest_main)
target_compile_definitions(test_mission PRIVATE
    FIELDNAV_SCENES_DIR="${PROJECT_SOURCE_DIR}/scenes"
    FIELDNAV_DATA_DIR="${PROJECT_SOURCE_DIR}/data"
)
fieldnav_enable_warnings(test_mission)
fieldnav_enable_sanitizers(test_mission)
fieldnav_copy_mujoco_runtime(test_mission)

add_test(NAME mission.integration COMMAND test_mission)
if(FIELDNAV_ENABLE_SANITIZERS)
    set_tests_properties(mission.integration PROPERTIES
        ENVIRONMENT "LSAN_OPTIONS=suppressions=${PROJECT_SOURCE_DIR}/src/sim/tests/lsan.supp"
    )
endif()
