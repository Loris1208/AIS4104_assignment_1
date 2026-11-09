add_executable(exercise_tg_presets
    ${CMAKE_SOURCE_DIR}/application/arm_main.cpp
    offered_arm.cpp
)

set(AIS4104_EXERCISE_TG_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/exercise-tg.xml" CACHE FILEPATH
    "The application document exercise_tg_presets reads at start-up")

target_link_libraries(exercise_tg_presets PRIVATE ais4104::arm_application ais4104::modules::modeling)

target_compile_definitions(exercise_tg_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_EXERCISE_TG_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(exercise_tg_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/exercise_tg_presets")

ais4104_target_warnings(exercise_tg_presets)

meios_target_deploy_resources(exercise_tg_presets
    RESOURCES ur_description
    PACKAGES urdf config meshes/ur3e
    SUBDIR urdf/ur_description
)

meios_target_deploy_resources(exercise_tg_presets
    RESOURCES course_licences
    SUBDIR licences
)

meios_target_deploy_resources(exercise_tg_presets
    RESOURCES course_models
    SUBDIR models
)
