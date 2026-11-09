add_executable(assignment_3_presets
    ${CMAKE_SOURCE_DIR}/application/arm_main.cpp
    offered_arm_student.cpp
)

set(AIS4104_ASSIGNMENT_3_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/assignment-3.xml" CACHE FILEPATH
    "The application document assignment_3_presets reads at start-up")

target_link_libraries(assignment_3_presets PRIVATE
    ais4104::arm_application ais4104::modules::rigid_motion ais4104::modules::forward_kinematics ais4104::modules::velocity_kinematics ais4104::modules::modeling ais4104::modules::inverse_kinematics ais4104::modules::trajectory_generation ais4104::modules::optimization)

target_compile_definitions(assignment_3_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_ASSIGNMENT_3_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(assignment_3_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/assignment_3_presets")

ais4104_target_warnings(assignment_3_presets)

meios_target_deploy_resources(assignment_3_presets
    RESOURCES kuka_experimental
    PACKAGES kuka_kr6_support kuka_resources
    SUBDIR urdf
)

meios_target_deploy_resources(assignment_3_presets
    RESOURCES lbr_med14_r820_description
    PACKAGES urdf config meshes
    SUBDIR urdf/lbr_med14_r820_description
)

meios_target_deploy_resources(assignment_3_presets
    RESOURCES course_licences
    SUBDIR licences
)

meios_target_deploy_resources(assignment_3_presets
    RESOURCES course_models
    SUBDIR models
)
