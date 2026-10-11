add_executable(assignment_2_presets
    ${CMAKE_SOURCE_DIR}/application/arm_main.cpp
    offered_arm_student.cpp
)

set(AIS4104_ASSIGNMENT_2_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/assignment-2.xml" CACHE FILEPATH
    "The application document assignment_2_presets reads at start-up")

target_link_libraries(assignment_2_presets PRIVATE
    ais4104::arm_application ais4104::modules::rigid_motion ais4104::modules::forward_kinematics ais4104::modules::velocity_kinematics ais4104::modules::modeling)

target_compile_definitions(assignment_2_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_ASSIGNMENT_2_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(assignment_2_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/assignment_2_presets")

ais4104_target_warnings(assignment_2_presets)

meios_target_deploy_resources(assignment_2_presets
    RESOURCES kuka_experimental
    PACKAGES kuka_kr6_support kuka_resources
    SUBDIR urdf
)

meios_target_deploy_resources(assignment_2_presets
    RESOURCES lbr_med14_r820_description
    PACKAGES urdf config meshes
    SUBDIR urdf/lbr_med14_r820_description
)

meios_target_deploy_resources(assignment_2_presets
    RESOURCES course_licences
    SUBDIR licences
)

meios_target_deploy_resources(assignment_2_presets
    RESOURCES course_models
    SUBDIR models
)
