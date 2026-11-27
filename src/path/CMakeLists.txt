# path : trajectoire de référence du rang (lecture du CSV), Path (erreur latérale, lookahead).

add_library(fieldnav_path STATIC
    reference.cpp
    path.cpp
)

target_include_directories(fieldnav_path
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(fieldnav_path PUBLIC cxx_std_17)
target_link_libraries(fieldnav_path PUBLIC fieldnav::odometry)

fieldnav_enable_warnings(fieldnav_path)
fieldnav_enable_sanitizers(fieldnav_path)

add_library(fieldnav::path ALIAS fieldnav_path)

if(FIELDNAV_BUILD_TESTS)
    add_subdirectory(tests)
endif()
