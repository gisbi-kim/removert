add_executable(removert_tests
    test_geometry.cpp
    test_range_image.cpp
    test_visibility.cpp
    test_revert.cpp
    test_pipeline_synthetic.cpp
    test_config_loader.cpp
    test_pcd_writer.cpp
    test_parity.cpp
)
target_link_libraries(removert_tests
    PRIVATE removert::core removert::io GTest::gtest GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(removert_tests)
