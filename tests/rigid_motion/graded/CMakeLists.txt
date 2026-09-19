add_executable(assignment_1_tests graded_frame_test.cpp graded_screw_test.cpp)

target_link_libraries(assignment_1_tests PRIVATE Catch2::Catch2WithMain praxis::rigid_motion praxis::evaluation ais4104::modules::rigid_motion)

target_include_directories(assignment_1_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

ais4104_target_warnings(assignment_1_tests)

catch_discover_tests(assignment_1_tests PROPERTIES LABELS graded)
