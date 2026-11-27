# Exécutables de FieldNav.

add_executable(fieldnav_run fieldnav_run.cpp)
target_link_libraries(fieldnav_run PRIVATE fieldnav::mission)
target_compile_definitions(fieldnav_run PRIVATE
    FIELDNAV_SCENES_DIR="${PROJECT_SOURCE_DIR}/scenes"
    FIELDNAV_DATA_DIR="${PROJECT_SOURCE_DIR}/data"
)
fieldnav_enable_warnings(fieldnav_run)
fieldnav_enable_sanitizers(fieldnav_run)
fieldnav_copy_mujoco_runtime(fieldnav_run)

if(FIELDNAV_BUILD_TESTS)
    # La ligne de commande elle-même : une courte mission réussit, une option invalide échoue.
    add_test(NAME apps.fieldnav_run_short COMMAND fieldnav_run --duration 5)
    add_test(NAME apps.fieldnav_run_bad_option COMMAND fieldnav_run --pose-source gps)
    set_tests_properties(apps.fieldnav_run_bad_option PROPERTIES WILL_FAIL TRUE)
    if(FIELDNAV_ENABLE_SANITIZERS)
        set_tests_properties(apps.fieldnav_run_short PROPERTIES
            ENVIRONMENT "LSAN_OPTIONS=suppressions=${PROJECT_SOURCE_DIR}/src/sim/tests/lsan.supp"
        )
    endif()
endif()
