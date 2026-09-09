file(GLOB ais4104_assignment_listfiles CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt")

foreach (ais4104_assignment_listfile IN LISTS ais4104_assignment_listfiles)
    get_filename_component(ais4104_assignment_directory "${ais4104_assignment_listfile}" DIRECTORY)
    get_filename_component(ais4104_assignment_name "${ais4104_assignment_directory}" NAME)
    message(STATUS "assignment: ${ais4104_assignment_name}")
    add_subdirectory("${ais4104_assignment_name}")
endforeach ()
