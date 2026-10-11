file(GLOB ais4104_velocity_kinematics_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(ais4104_modules_velocity_kinematics STATIC ${ais4104_velocity_kinematics_sources})

add_library(ais4104::modules::velocity_kinematics ALIAS ais4104_modules_velocity_kinematics)

target_include_directories(ais4104_modules_velocity_kinematics PUBLIC "${CMAKE_SOURCE_DIR}/modules")

target_link_libraries(ais4104_modules_velocity_kinematics PUBLIC praxis::manipulator praxis::rigid_motion)

target_compile_features(ais4104_modules_velocity_kinematics PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_modules_velocity_kinematics)
