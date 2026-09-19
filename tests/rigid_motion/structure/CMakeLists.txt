add_executable(assignment_1_structure_tests rigid_motion_test.cpp slot_order_test.cpp task_marker_test.cpp)

target_link_libraries(assignment_1_structure_tests PRIVATE Catch2::Catch2WithMain praxis::rigid_motion praxis::evaluation ais4104::modules::rigid_motion ais4104::slot_report)

target_include_directories(assignment_1_structure_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_compile_definitions(assignment_1_structure_tests PRIVATE AIS4104_MODULE_SOURCE_DIRECTORY="${PROJECT_SOURCE_DIR}/modules/rigid_motion")

ais4104_target_warnings(assignment_1_structure_tests)

catch_discover_tests(assignment_1_structure_tests)
