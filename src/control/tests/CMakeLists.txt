# Tests du pure pursuit (GoogleTest).

add_executable(test_control
    test_pure_pursuit.cpp
)
target_link_libraries(test_control PRIVATE fieldnav::control GTest::gtest_main)
fieldnav_enable_warnings(test_control)
fieldnav_enable_sanitizers(test_control)

add_test(NAME control.unit COMMAND test_control)
