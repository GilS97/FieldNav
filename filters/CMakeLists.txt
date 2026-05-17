# Phase 3: placeholder. Implementations land later.
add_library(fieldnav_filters INTERFACE)
target_include_directories(fieldnav_filters
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(fieldnav_filters INTERFACE fieldnav::core)
add_library(fieldnav::filters ALIAS fieldnav_filters)
