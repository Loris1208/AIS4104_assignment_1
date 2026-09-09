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
