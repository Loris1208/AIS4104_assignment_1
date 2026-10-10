add_library(ais4104_slot_report STATIC
    slot_report.h
    slot_report.cpp
)

add_library(ais4104::slot_report ALIAS ais4104_slot_report)

target_include_directories(ais4104_slot_report PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(ais4104_slot_report PUBLIC praxis::rigid_motion praxis::evaluation spdlog::spdlog)

target_compile_features(ais4104_slot_report PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_slot_report)

add_library(ais4104_application STATIC
    motions.h

    offered.h
    offered.cpp
    write_back.h
    write_back.cpp
    configuration.h
    configuration.cpp
    document_store.h
    document_store.cpp
)

add_library(ais4104::application ALIAS ais4104_application)

target_include_directories(ais4104_application PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(ais4104_application PUBLIC ais4104::slot_report praxis::praxis praxis::rigid_presets praxis::rigid_motion praxis::config spdlog::spdlog)

target_compile_features(ais4104_application PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_application)

add_library(ais4104_arm_application STATIC
    arm.h

    offered_arms.h
    offered_arms.cpp
    arm_keyspace.h
    arm_keyspace.cpp
    arm_slot_report.h
    arm_slot_report.cpp
    arm_render_loop.h
    arm_render_loop.cpp
    arm_windows.h
    arm_windows.cpp
    arm_tool_seat.cpp
    arm_pose_frames.cpp
    hand_derived.h
    hand_derived_tool.cpp
    kept_chain.h
    kept_chain.cpp
    jacobian_displacement.h
    jacobian_displacement.cpp
    jacobian_jog_window.h
    jacobian_jog_window.cpp
)

add_library(ais4104::arm_application ALIAS ais4104_arm_application)

target_include_directories(ais4104_arm_application PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(ais4104_arm_application PUBLIC ais4104::application ais4104::modules::modeling praxis::manipulator_presets praxis::manipulator praxis::trajectory praxis::evaluation
    spdlog::spdlog)

target_compile_definitions(ais4104_arm_application PRIVATE AIS4104_PREVIEW_FRAMES=${AIS4104_PREVIEW_FRAMES})

target_compile_features(ais4104_arm_application PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_arm_application)
