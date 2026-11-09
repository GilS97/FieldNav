# Tests de la fusion (GoogleTest).

add_executable(test_estimation
    test_complementary.cpp
)
target_link_libraries(test_estimation PRIVATE fieldnav::estimation GTest::gtest_main)
fieldnav_enable_warnings(test_estimation)
fieldnav_enable_sanitizers(test_estimation)

add_test(NAME estimation.unit COMMAND test_estimation)
