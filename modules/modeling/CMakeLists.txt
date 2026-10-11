file(GLOB ais4104_modeling_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(ais4104_modules_modeling STATIC ${ais4104_modeling_sources})

add_library(ais4104::modules::modeling ALIAS ais4104_modules_modeling)

target_include_directories(ais4104_modules_modeling PUBLIC "${CMAKE_SOURCE_DIR}/modules")

target_link_libraries(ais4104_modules_modeling PUBLIC praxis::rigid_motion)

target_compile_features(ais4104_modules_modeling PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_modules_modeling)
