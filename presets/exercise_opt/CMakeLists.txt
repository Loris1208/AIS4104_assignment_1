add_executable(exercise_opt_presets
    ${CMAKE_SOURCE_DIR}/application/arm_main.cpp
    offered_arm.cpp
)

set(AIS4104_EXERCISE_OPT_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/exercise-opt.xml" CACHE FILEPATH
    "The application document exercise_opt_presets reads at start-up")

target_link_libraries(exercise_opt_presets PRIVATE ais4104::arm_application ais4104::modules::modeling)

target_compile_definitions(exercise_opt_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_EXERCISE_OPT_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(exercise_opt_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/exercise_opt_presets")

ais4104_target_warnings(exercise_opt_presets)
