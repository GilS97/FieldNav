# Reusable function to apply strict warnings to a target.
#
# Usage:
#   fieldnav_enable_warnings(my_target)

function(fieldnav_enable_warnings target)
    if(NOT FIELDNAV_ENABLE_WARNINGS)
        return()
    endif()

    set(GCC_CLANG_FLAGS
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
    )

    set(MSVC_FLAGS /W4 /permissive-)

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:${GCC_CLANG_FLAGS}>
            $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion>
        )
    elseif(MSVC)
        target_compile_options(${target} PRIVATE ${MSVC_FLAGS})
    endif()

    if(FIELDNAV_WARNINGS_AS_ERRORS)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${target} PRIVATE -Werror)
        elseif(MSVC)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    endif()
endfunction()
