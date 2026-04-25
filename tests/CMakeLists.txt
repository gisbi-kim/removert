set(REMOVERT_TEST_SOURCES
    test_geometry.cpp
    test_range_image.cpp
    test_visibility.cpp
    test_revert.cpp
    test_pipeline_synthetic.cpp
    test_config_loader.cpp
    test_pcd_writer.cpp
    test_parity.cpp
)

if(REMOVERT_WITH_HDF5)
    list(APPEND REMOVERT_TEST_SOURCES test_hdf5_writer.cpp)
endif()

add_executable(removert_tests ${REMOVERT_TEST_SOURCES})
target_link_libraries(removert_tests
    PRIVATE removert::core removert::io GTest::gtest GTest::gtest_main)
if(REMOVERT_WITH_HDF5)
    target_include_directories(removert_tests PRIVATE ${HDF5_INCLUDE_DIRS})
    target_link_libraries(removert_tests PRIVATE ${HDF5_CXX_LIBRARIES})
endif()

include(GoogleTest)
gtest_discover_tests(removert_tests)
