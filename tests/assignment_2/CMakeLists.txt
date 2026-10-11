add_executable(assignment_2_tests described_chains.cpp graded_home_test.cpp graded_screw_test.cpp graded_jacobian_test.cpp graded_kinematics_test.cpp graded_tool_test.cpp manipulator_columns.cpp manipulator_rows.cpp)

target_link_libraries(assignment_2_tests PRIVATE Catch2::Catch2WithMain ais4104::arm_application ais4104::modules::modeling ais4104::modules::forward_kinematics ais4104::modules::velocity_kinematics praxis::evaluation)

target_compile_definitions(assignment_2_tests PRIVATE AIS4104_DOCUMENT_DIRECTORY="${PROJECT_SOURCE_DIR}/application/documents"
    AIS4104_DEPLOYED_DIRECTORY="$<TARGET_FILE_DIR:assignment_2_tests>")

ais4104_target_warnings(assignment_2_tests)

meios_target_deploy_resources(assignment_2_tests
    RESOURCES kuka_experimental
    PACKAGES kuka_kr6_support kuka_resources
    SUBDIR urdf
)

meios_target_deploy_resources(assignment_2_tests
    RESOURCES lbr_med14_r820_description
    PACKAGES urdf config meshes
    SUBDIR urdf/lbr_med14_r820_description
)

meios_target_deploy_resources(assignment_2_tests
    RESOURCES course_licences
    SUBDIR licences
)

meios_target_deploy_resources(assignment_2_tests
    RESOURCES course_models
    SUBDIR models
)

catch_discover_tests(assignment_2_tests PROPERTIES LABELS graded)
