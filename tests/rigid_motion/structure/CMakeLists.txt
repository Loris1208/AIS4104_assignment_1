add_executable(rigid_motion_structure_tests rigid_motion_test.cpp slot_order_test.cpp task_marker_test.cpp)

target_link_libraries(rigid_motion_structure_tests PRIVATE Catch2::Catch2WithMain praxis::rigid_motion praxis::evaluation ais4104::modules::rigid_motion ais4104::slot_report)

target_include_directories(rigid_motion_structure_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_compile_definitions(rigid_motion_structure_tests PRIVATE AIS4104_MODULE_SOURCE_DIRECTORY="${PROJECT_SOURCE_DIR}/modules/rigid_motion")

ais4104_target_warnings(rigid_motion_structure_tests)

catch_discover_tests(rigid_motion_structure_tests)
