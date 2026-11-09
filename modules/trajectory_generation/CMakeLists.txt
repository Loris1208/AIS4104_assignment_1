file(GLOB ais4104_trajectory_generation_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(ais4104_modules_trajectory_generation STATIC ${ais4104_trajectory_generation_sources})

add_library(ais4104::modules::trajectory_generation ALIAS ais4104_modules_trajectory_generation)

target_include_directories(ais4104_modules_trajectory_generation PUBLIC "${CMAKE_SOURCE_DIR}/modules")

target_link_libraries(ais4104_modules_trajectory_generation PUBLIC praxis::trajectory praxis::rigid_motion)

target_compile_features(ais4104_modules_trajectory_generation PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_modules_trajectory_generation)
