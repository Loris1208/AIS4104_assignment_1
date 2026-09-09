option(AIS4104_WARNINGS_AS_ERRORS "Promote this project's own compiler warnings to errors; the shipped default preset leaves it off so an untidy build still succeeds" OFF)

function(ais4104_target_warnings TARGET)
    target_compile_options(${TARGET} PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4;/utf-8>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall;-Wextra;-Wpedantic>
        $<$<AND:$<BOOL:${AIS4104_WARNINGS_AS_ERRORS}>,$<CXX_COMPILER_ID:MSVC>>:/WX>
        $<$<AND:$<BOOL:${AIS4104_WARNINGS_AS_ERRORS}>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-Werror>
    )

    target_compile_definitions(${TARGET} PRIVATE $<$<PLATFORM_ID:Windows>:NOMINMAX;WIN32_LEAN_AND_MEAN>)
endfunction()
