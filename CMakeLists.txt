cmake_minimum_required(VERSION 3.20)

project(FieldNav
    VERSION 0.1.0
    DESCRIPTION "Open-source library for outdoor robot localization"
    HOMEPAGE_URL "https://github.com/YOUR-USER/fieldnav"
    LANGUAGES C CXX
)

# ---- Standards ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# ---- Top-level only: export compile_commands.json for clangd, IDEs ----
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# ---- Options ----
option(FIELDNAV_BUILD_TESTS         "Build all tests"                ON)
option(FIELDNAV_BUILD_C_LEGACY      "Build Phase 1 C matrix lib"     ON)
option(FIELDNAV_BUILD_CORE          "Build Phase 2 C++ core lib"     ON)
option(FIELDNAV_BUILD_FILTERS       "Build Phase 3 filters lib"      OFF)
option(FIELDNAV_ENABLE_WARNINGS     "Enable strict compiler warnings" ON)
option(FIELDNAV_ENABLE_SANITIZERS   "Enable AddressSanitizer + UBSan" OFF)
option(FIELDNAV_WARNINGS_AS_ERRORS  "Treat warnings as errors"       OFF)

# ---- CMake module path ----
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)
include(Sanitizers)

# ---- Testing infrastructure ----
if(FIELDNAV_BUILD_TESTS)
    enable_testing()
    include(FetchContent)
    set(FETCHCONTENT_QUIET FALSE)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
        SYSTEM
    )
    # Avoid installing gtest globally
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# ---- Modules ----
if(FIELDNAV_BUILD_C_LEGACY)
    add_subdirectory(c_legacy)
endif()

if(FIELDNAV_BUILD_CORE)
    add_subdirectory(core)
endif()

if(FIELDNAV_BUILD_FILTERS)
    add_subdirectory(filters)
endif()

# ---- Summary ----
message(STATUS "")
message(STATUS "============== FieldNav build configuration ==============")
message(STATUS "  Version:           ${PROJECT_VERSION}")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "  C compiler:        ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}")
message(STATUS "  C++ compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C standard:        C${CMAKE_C_STANDARD}")
message(STATUS "  C++ standard:      C++${CMAKE_CXX_STANDARD}")
message(STATUS "  ")
message(STATUS "  Tests:             ${FIELDNAV_BUILD_TESTS}")
message(STATUS "  C legacy module:   ${FIELDNAV_BUILD_C_LEGACY}")
message(STATUS "  Core module:       ${FIELDNAV_BUILD_CORE}")
message(STATUS "  Filters module:    ${FIELDNAV_BUILD_FILTERS}")
message(STATUS "  Strict warnings:   ${FIELDNAV_ENABLE_WARNINGS}")
message(STATUS "  Sanitizers:        ${FIELDNAV_ENABLE_SANITIZERS}")
message(STATUS "==========================================================")
message(STATUS "")
