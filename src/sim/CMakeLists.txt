# sim : simulation physique MuJoCo du robot différentiel (pose vraie, vitesses de roues).

add_library(fieldnav_sim STATIC
    diff_drive_sim.cpp
)

target_include_directories(fieldnav_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_sim PUBLIC cxx_std_17)
target_link_libraries(fieldnav_sim
    PUBLIC  fieldnav::odometry
    PRIVATE mujoco::mujoco
)

fieldnav_enable_warnings(fieldnav_sim)
fieldnav_enable_sanitizers(fieldnav_sim)

add_library(fieldnav::sim ALIAS fieldnav_sim)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
