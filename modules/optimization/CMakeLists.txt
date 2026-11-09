file(GLOB ais4104_optimization_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(ais4104_modules_optimization STATIC ${ais4104_optimization_sources})

add_library(ais4104::modules::optimization ALIAS ais4104_modules_optimization)

target_include_directories(ais4104_modules_optimization PUBLIC "${CMAKE_SOURCE_DIR}/modules")

target_link_libraries(ais4104_modules_optimization PUBLIC praxis::optimization)

target_compile_features(ais4104_modules_optimization PUBLIC cxx_std_20)

ais4104_target_warnings(ais4104_modules_optimization)
