# Tests de la simulation MuJoCo (GoogleTest).

add_executable(test_diff_drive_sim
    test_diff_drive_sim.cpp
)
target_link_libraries(test_diff_drive_sim PRIVATE fieldnav::sim GTest::gtest_main)
target_compile_definitions(test_diff_drive_sim PRIVATE
    FIELDNAV_SCENES_DIR="${PROJECT_SOURCE_DIR}/scenes"
)
fieldnav_enable_warnings(test_diff_drive_sim)
fieldnav_enable_sanitizers(test_diff_drive_sim)
fieldnav_copy_mujoco_runtime(test_diff_drive_sim)

add_test(NAME sim.unit COMMAND test_diff_drive_sim)

# Intégration : sim → bruit des roues → (v, ω) → odométrie, sur 60 s simulées.
add_executable(test_odometry_in_sim
    test_odometry_in_sim.cpp
)
target_link_libraries(test_odometry_in_sim PRIVATE
    fieldnav::sim fieldnav::sensors fieldnav::odometry GTest::gtest_main
)
target_compile_definitions(test_odometry_in_sim PRIVATE
    FIELDNAV_SCENES_DIR="${PROJECT_SOURCE_DIR}/scenes"
)
fieldnav_enable_warnings(test_odometry_in_sim)
fieldnav_enable_sanitizers(test_odometry_in_sim)
fieldnav_copy_mujoco_runtime(test_odometry_in_sim)

add_test(NAME sim.odometry_integration COMMAND test_odometry_in_sim)

# MuJoCo précompilé n'est pas instrumenté : on ignore les fuites internes à libmujoco
# (et seulement celles-là) sous LeakSanitizer.
if(FIELDNAV_ENABLE_SANITIZERS)
    set_tests_properties(sim.unit sim.odometry_integration PROPERTIES
        ENVIRONMENT "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp"
    )
endif()
