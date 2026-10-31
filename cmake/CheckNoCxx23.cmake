# Échoue si un fichier de libs/sensor_utils contient encore une construction C++23.
# Usage : cmake -DSCAN_DIR=<dossier> -P check_no_cxx23.cmake

if(NOT SCAN_DIR)
    message(FATAL_ERROR "SCAN_DIR non défini")
endif()

# Motifs repérés lors de l'import ; compléter si la relecture en trouve d'autres.
set(PATTERNS
    "std::expected" "std::unexpected" "<expected>"
    "std::print" "<print>"
    "this auto" "cxx_std_23" "CMAKE_CXX_STANDARD 23" "C\\+\\+23"
)

file(GLOB_RECURSE FILES
    "${SCAN_DIR}/*.hpp" "${SCAN_DIR}/*.h" "${SCAN_DIR}/*.cpp" "${SCAN_DIR}/CMakeLists.txt"
)

set(FOUND 0)
foreach(f IN LISTS FILES)
    file(STRINGS "${f}" lines)
    set(lineno 0)
    foreach(line IN LISTS lines)
        math(EXPR lineno "${lineno} + 1")
        foreach(p IN LISTS PATTERNS)
            if(line MATCHES "${p}")
                message(STATUS "${f}:${lineno}: ${line}")
                set(FOUND 1)
            endif()
        endforeach()
    endforeach()
endforeach()

if(FOUND)
    message(FATAL_ERROR "Constructions C++23 restantes dans ${SCAN_DIR}")
endif()
