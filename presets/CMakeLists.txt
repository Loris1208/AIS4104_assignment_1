file(GLOB ais4104_preset_listfiles CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt")

foreach (ais4104_preset_listfile IN LISTS ais4104_preset_listfiles)
    get_filename_component(ais4104_preset_directory "${ais4104_preset_listfile}" DIRECTORY)
    get_filename_component(ais4104_preset_name "${ais4104_preset_directory}" NAME)
    message(STATUS "preset: ${ais4104_preset_name}")
    add_subdirectory("${ais4104_preset_name}")
endforeach ()
