file(GLOB ais4104_suite_listfiles CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt")

foreach (ais4104_suite_listfile IN LISTS ais4104_suite_listfiles)
    get_filename_component(ais4104_suite_directory "${ais4104_suite_listfile}" DIRECTORY)
    get_filename_component(ais4104_suite_name "${ais4104_suite_directory}" NAME)
    message(STATUS "suite: ${ais4104_suite_name}")
    add_subdirectory("${ais4104_suite_name}")
endforeach ()

function(ais4104_test_executables directory result)
    set(executables "")
    get_property(targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach (target IN LISTS targets)
        get_target_property(type ${target} TYPE)
        if (type STREQUAL "EXECUTABLE")
            list(APPEND executables "$<TARGET_FILE:${target}>")
        endif ()
    endforeach ()

    get_property(subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
    foreach (subdirectory IN LISTS subdirectories)
        ais4104_test_executables("${subdirectory}" nested)
        list(APPEND executables ${nested})
    endforeach ()
    set(${result} "${executables}" PARENT_SCOPE)
endfunction()

ais4104_test_executables("${CMAKE_CURRENT_SOURCE_DIR}" ais4104_suite_executables)

add_test(NAME "every test executable lists each test name on one line"
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_SOURCE_DIR}/single_line_names.cmake" -- ${ais4104_suite_executables})
