file(GLOB ais4104_suite_listfiles CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt")

foreach (ais4104_suite_listfile IN LISTS ais4104_suite_listfiles)
    get_filename_component(ais4104_suite_directory "${ais4104_suite_listfile}" DIRECTORY)
    get_filename_component(ais4104_suite_name "${ais4104_suite_directory}" NAME)
    message(STATUS "suite: ${ais4104_suite_name}")
    add_subdirectory("${ais4104_suite_name}")
endforeach ()
