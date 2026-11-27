# control : pure pursuit (suivi du chemin).

add_library(fieldnav_control STATIC
    pure_pursuit.cpp
)

target_include_directories(fieldnav_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_control PUBLIC cxx_std_17)
target_link_libraries(fieldnav_control PUBLIC fieldnav::path fieldnav::odometry)

fieldnav_enable_warnings(fieldnav_control)
fieldnav_enable_sanitizers(fieldnav_control)

add_library(fieldnav::control ALIAS fieldnav_control)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
