add_executable(assignment_1_presets
    ${CMAKE_SOURCE_DIR}/application/main.cpp
    offered_motions.cpp
)

set(AIS4104_ASSIGNMENT_1_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/rigid-motions.xml" CACHE FILEPATH
    "The application document assignment_1_presets reads at start-up")

target_link_libraries(assignment_1_presets PRIVATE ais4104::application ais4104::modules::rigid_motion)

target_compile_definitions(assignment_1_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_ASSIGNMENT_1_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(assignment_1_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/assignment_1_presets")

ais4104_target_warnings(assignment_1_presets)
