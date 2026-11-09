add_executable(assignment_3_tg_baseline
    ${CMAKE_SOURCE_DIR}/application/arm_main.cpp
    offered_arm_baseline.cpp
)

set(AIS4104_ASSIGNMENT_3_TG_BASELINE_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/assignment-3-tg-baseline.xml" CACHE FILEPATH
    "The application document assignment_3_tg_baseline reads at start-up")

target_link_libraries(assignment_3_tg_baseline PRIVATE ais4104::arm_application ais4104::modules::trajectory_generation ais4104::modules::modeling)

target_compile_definitions(assignment_3_tg_baseline PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_ASSIGNMENT_3_TG_BASELINE_CONFIGURATION}")

praxis_target_persistent_directory(assignment_3_tg_baseline DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/assignment_3_tg_baseline")

ais4104_target_warnings(assignment_3_tg_baseline)

meios_target_deploy_resources(assignment_3_tg_baseline
    RESOURCES kuka_experimental
    PACKAGES kuka_kr6_support kuka_resources
    SUBDIR urdf
)

meios_target_deploy_resources(assignment_3_tg_baseline
    RESOURCES lbr_med14_r820_description
    PACKAGES urdf config meshes
    SUBDIR urdf/lbr_med14_r820_description
)

meios_target_deploy_resources(assignment_3_tg_baseline
    RESOURCES course_licences
    SUBDIR licences
)

meios_target_deploy_resources(assignment_3_tg_baseline
    RESOURCES course_models
    SUBDIR models
)
