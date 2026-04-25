add_executable(removert_cli removert_cli.cpp)
target_link_libraries(removert_cli PRIVATE removert::core removert::io)
set_target_properties(removert_cli PROPERTIES OUTPUT_NAME removert_cli)
