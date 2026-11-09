# Tests des capteurs bruités (GoogleTest).

add_executable(test_sensors
    test_wheel_noise.cpp
)
target_link_libraries(test_sensors PRIVATE fieldnav::sensors GTest::gtest_main)
fieldnav_enable_warnings(test_sensors)
fieldnav_enable_sanitizers(test_sensors)

add_test(NAME sensors.unit COMMAND test_sensors)
