file(GLOB ais4104_rigid_motion_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(ais4104_modules_rigid_motion STATIC ${ais4104_rigid_motion_sources})

add_library(ais4104::modules::rigid_motion ALIAS ais4104_modules_rigid_motion)

target_include_directories(ais4104_modules_rigid_motion PUBLIC "${CMAKE_SOURCE_DIR}/modules")

target_link_libraries(ais4104_modules_rigid_motion PUBLIC praxis::rigid_motion)

target_compile_features(ais4104_modules_rigid_motion PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_modules_rigid_motion)
