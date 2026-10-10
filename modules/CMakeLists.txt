file(GLOB ais4104_module_listfiles CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt")

foreach (ais4104_module_listfile IN LISTS ais4104_module_listfiles)
    get_filename_component(ais4104_module_directory "${ais4104_module_listfile}" DIRECTORY)
    get_filename_component(ais4104_module_name "${ais4104_module_directory}" NAME)
    message(STATUS "module: ${ais4104_module_name}")
    add_subdirectory("${ais4104_module_name}")
endforeach ()
