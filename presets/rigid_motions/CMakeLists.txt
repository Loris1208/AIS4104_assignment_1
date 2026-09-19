add_executable(rigid_motion_presets
    ${CMAKE_SOURCE_DIR}/application/main.cpp
    offered_motions.cpp
)

set(AIS4104_RIGID_MOTION_PRESETS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/rigid-motions.xml" CACHE FILEPATH
    "The application document rigid_motion_presets reads at start-up")

target_link_libraries(rigid_motion_presets PRIVATE ais4104::application)

target_compile_definitions(rigid_motion_presets PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_RIGID_MOTION_PRESETS_CONFIGURATION}")

praxis_target_persistent_directory(rigid_motion_presets DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/rigid_motion_presets")

ais4104_target_warnings(rigid_motion_presets)
