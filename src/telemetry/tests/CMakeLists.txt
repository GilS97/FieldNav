# Tests du TelemetryWriter (GoogleTest).

add_executable(test_telemetry
    test_telemetry_writer.cpp
)
target_link_libraries(test_telemetry PRIVATE fieldnav::telemetry GTest::gtest_main)
fieldnav_enable_warnings(test_telemetry)
fieldnav_enable_sanitizers(test_telemetry)

add_test(NAME telemetry.unit COMMAND test_telemetry)
