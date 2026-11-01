# Tests de l'odométrie (GoogleTest, fourni par le CMakeLists racine via FetchContent).

add_executable(test_odometry
    test_odometry.cpp
)
target_link_libraries(test_odometry PRIVATE fieldnav::odometry GTest::gtest_main)
fieldnav_enable_warnings(test_odometry)
fieldnav_enable_sanitizers(test_odometry)

add_test(NAME odometry.unit COMMAND test_odometry)
