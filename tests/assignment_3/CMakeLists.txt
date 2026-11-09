file(GLOB ais4104_assignment_3_tests_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(assignment_3_tests ${ais4104_assignment_3_tests_sources})

target_link_libraries(assignment_3_tests PRIVATE Catch2::Catch2WithMain ais4104::modules::inverse_kinematics ais4104::modules::trajectory_generation ais4104::modules::optimization praxis::manipulator praxis::trajectory praxis::optimization praxis::rigid_motion praxis::evaluation)

ais4104_target_warnings(assignment_3_tests)

catch_discover_tests(assignment_3_tests PROPERTIES LABELS graded)
