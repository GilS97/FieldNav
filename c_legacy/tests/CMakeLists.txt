# Phase 1 tests use a minimal C assertion framework
# (educational: shows ctest can run any executable).

add_executable(test_matrix test_matrix.c)
target_link_libraries(test_matrix PRIVATE fieldnav::c_legacy)
fieldnav_enable_warnings(test_matrix)
fieldnav_enable_sanitizers(test_matrix)

add_test(NAME c_legacy.matrix COMMAND test_matrix)
