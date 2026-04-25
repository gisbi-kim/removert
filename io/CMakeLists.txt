# io/ — dataset readers + Config loader. Depends only on `core` (no GPU).

set(REMOVERT_IO_SOURCES
    src/config_loader.cpp
    src/kitti_dir_reader.cpp
    src/pcd_writer.cpp        # always available, no PCL dependency
)

if(REMOVERT_WITH_SQLITE)
    list(APPEND REMOVERT_IO_SOURCES src/sqlite_reader.cpp)
endif()

if(REMOVERT_WITH_HDF5)
    list(APPEND REMOVERT_IO_SOURCES src/hdf5_reader.cpp src/hdf5_writer.cpp)
endif()

add_library(removert_io STATIC ${REMOVERT_IO_SOURCES})
add_library(removert::io ALIAS removert_io)

target_include_directories(removert_io
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(removert_io
    PUBLIC  removert::core nlohmann_json::nlohmann_json
)

if(REMOVERT_WITH_SQLITE)
    target_link_libraries(removert_io PRIVATE SQLite::SQLite3)
    target_compile_definitions(removert_io PUBLIC REMOVERT_WITH_SQLITE=1)
endif()

if(REMOVERT_WITH_HDF5)
    target_include_directories(removert_io PRIVATE ${HDF5_INCLUDE_DIRS})
    target_link_libraries(removert_io PRIVATE ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})
    target_compile_definitions(removert_io PUBLIC REMOVERT_WITH_HDF5=1)
endif()

