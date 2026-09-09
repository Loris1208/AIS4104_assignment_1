add_executable(assignment_rigid_motions
    ${CMAKE_SOURCE_DIR}/application/main.cpp
    offered_motions.cpp
)

set(AIS4104_ASSIGNMENT_RIGID_MOTIONS_CONFIGURATION "${CMAKE_SOURCE_DIR}/application/documents/rigid-motions.xml" CACHE FILEPATH
    "The application document assignment_rigid_motions reads at start-up")

target_link_libraries(assignment_rigid_motions PRIVATE ais4104::application ais4104::modules::rigid_motion)

target_compile_definitions(assignment_rigid_motions PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES}
    AIS4104_APPLICATION_CONFIGURATION="${AIS4104_ASSIGNMENT_RIGID_MOTIONS_CONFIGURATION}")

praxis_target_persistent_directory(assignment_rigid_motions DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/assignment_rigid_motions")

ais4104_target_warnings(assignment_rigid_motions)
