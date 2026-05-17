# Reusable function to enable sanitizers on a target.
#
# Usage:
#   fieldnav_enable_sanitizers(my_target)
#
# When FIELDNAV_ENABLE_SANITIZERS is ON, applies AddressSanitizer +
# UndefinedBehaviorSanitizer to the target.

function(fieldnav_enable_sanitizers target)
    if(NOT FIELDNAV_ENABLE_SANITIZERS)
        return()
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
            -fno-sanitize-recover=all
        )
        target_link_options(${target} PRIVATE
            -fsanitize=address,undefined
        )
    endif()
endfunction()
