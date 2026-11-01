# odometry : pose 2D et odométrie différentielle (Euler), en C++17 comme le reste de FieldNav.

add_library(fieldnav_odometry STATIC
    odometry.cpp
)

target_include_directories(fieldnav_odometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_odometry PUBLIC cxx_std_17)

fieldnav_enable_warnings(fieldnav_odometry)
fieldnav_enable_sanitizers(fieldnav_odometry)

add_library(fieldnav::odometry ALIAS fieldnav_odometry)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
