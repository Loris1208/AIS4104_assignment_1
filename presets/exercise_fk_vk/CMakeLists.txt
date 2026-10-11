add_executable(exercise_fk_vk_presets
    ${CMAKE_SOURCE_DIR}/application/arm_main.cpp
    offered_arm.cpp
)

set(AIS4104_EXERCISE_FK_VK_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/exercise-fk-vk.xml" CACHE FILEPATH
    "The application document exercise_fk_vk_presets reads at start-up")

target_link_libraries(exercise_fk_vk_presets PRIVATE ais4104::arm_application ais4104::modules::modeling)

target_compile_definitions(exercise_fk_vk_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_EXERCISE_FK_VK_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(exercise_fk_vk_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/exercise_fk_vk_presets")

ais4104_target_warnings(exercise_fk_vk_presets)

meios_target_deploy_resources(exercise_fk_vk_presets
    RESOURCES kuka_experimental
    PACKAGES kuka_kr6_support kuka_resources
    SUBDIR urdf
)

meios_target_deploy_resources(exercise_fk_vk_presets
    RESOURCES ur_description
    PACKAGES urdf config meshes/ur3e
    SUBDIR urdf/ur_description
)

meios_target_deploy_resources(exercise_fk_vk_presets
    RESOURCES lbr_med14_r820_description
    PACKAGES urdf config meshes
    SUBDIR urdf/lbr_med14_r820_description
)

meios_target_deploy_resources(exercise_fk_vk_presets
    RESOURCES course_licences
    SUBDIR licences
)

meios_target_deploy_resources(exercise_fk_vk_presets
    RESOURCES course_models
    SUBDIR models
)
