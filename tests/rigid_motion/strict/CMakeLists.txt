add_executable(rigid_motion_strict_tests strict_test.cpp)

target_link_libraries(rigid_motion_strict_tests PRIVATE Catch2::Catch2WithMain praxis::rigid_motion praxis::evaluation ais4104::modules::rigid_motion)

target_include_directories(rigid_motion_strict_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

ais4104_target_warnings(rigid_motion_strict_tests)

catch_discover_tests(rigid_motion_strict_tests)
