# Télécharge MuJoCo précompilé (release officielle google-deepmind/mujoco) et crée la cible
# importée mujoco::mujoco. Rien n'est compilé : on utilise la bibliothèque partagée fournie.
#
# Usage :
#   include(FetchMuJoCo)
#   target_link_libraries(ma_cible PRIVATE mujoco::mujoco)
#   fieldnav_copy_mujoco_runtime(mon_executable)   # Windows : copie mujoco.dll à côté de l'exe

include(FetchContent)

set(FIELDNAV_MUJOCO_VERSION "3.16.0")
set(_mj_base "https://github.com/google-deepmind/mujoco/releases/download/${FIELDNAV_MUJOCO_VERSION}")

if(WIN32)
    set(_mj_url  "${_mj_base}/mujoco-${FIELDNAV_MUJOCO_VERSION}-windows-x86_64.zip")
    set(_mj_hash "0e25e6fc21fa8d6d214481d9d8a50e2649a7f1fba39c0176be4b8198cc6d84c3")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(_mj_url  "${_mj_base}/mujoco-${FIELDNAV_MUJOCO_VERSION}-linux-x86_64.tar.gz")
    set(_mj_hash "139577c541d43716a63992ab698cbc7b09788b1d547fb82b682d59b40ef49b9d")
else()
    message(FATAL_ERROR "MuJoCo précompilé : plateforme non prévue (${CMAKE_SYSTEM_NAME} "
                        "${CMAKE_SYSTEM_PROCESSOR}). Désactiver avec -DFIELDNAV_BUILD_SIM=OFF.")
endif()

# Le CMake de WinLibs (GnuTLS) n'a pas de certificats racines : sans eux, le téléchargement
# HTTPS échoue (« certificate signer not trusted »). On réutilise ceux installés avec Git.
if(WIN32 AND NOT CMAKE_TLS_CAINFO)
    find_file(FIELDNAV_CA_BUNDLE
        NAMES ca-bundle.crt
        PATHS
            "$ENV{SSL_CERT_FILE}/.."
            "C:/Program Files/Git/usr/ssl/certs"
            "C:/Program Files/Git/mingw64/etc/ssl/certs"
            "C:/Program Files/Git/mingw64/ssl/certs"
            "C:/msys64/usr/ssl/certs"
        NO_DEFAULT_PATH
    )
    if(FIELDNAV_CA_BUNDLE)
        set(CMAKE_TLS_CAINFO "${FIELDNAV_CA_BUNDLE}")
    else()
        message(WARNING "Aucun ca-bundle.crt trouvé : si le téléchargement de MuJoCo échoue, "
                        "relancer avec -DCMAKE_TLS_CAINFO=<chemin vers ca-bundle.crt>.")
    endif()
endif()

FetchContent_Declare(mujoco_bin
    URL      "${_mj_url}"
    URL_HASH SHA256=${_mj_hash}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    # L'archive n'a pas de CMakeLists.txt : on la télécharge et l'extrait seulement.
    SOURCE_SUBDIR "no-cmake-here"
)
FetchContent_MakeAvailable(mujoco_bin)
set(_mj_dir "${mujoco_bin_SOURCE_DIR}")

add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
# Les en-têtes d'une cible importée sont traités comme « système » : pas de warnings de MuJoCo.
set_target_properties(mujoco::mujoco PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_mj_dir}/include"
)

if(WIN32)
    # mujoco.lib est une bibliothèque d'import MSVC ; le linker de MinGW la lit sans problème
    # (vérifié avec MinGW g++ 16 et MuJoCo 3.16.0 le 1er novembre 2026).
    set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION "${_mj_dir}/bin/mujoco.dll"
        IMPORTED_IMPLIB   "${_mj_dir}/lib/mujoco.lib"
    )
else()
    if(EXISTS "${_mj_dir}/lib/libmujoco.so")
        set(_mj_so "${_mj_dir}/lib/libmujoco.so")
    else()
        file(GLOB _mj_so_candidates "${_mj_dir}/lib/libmujoco.so*")
        list(GET _mj_so_candidates 0 _mj_so)
    endif()
    set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_LOCATION "${_mj_so}")
endif()

message(STATUS "MuJoCo ${FIELDNAV_MUJOCO_VERSION} : ${_mj_dir}")

# Sous Windows, une DLL doit se trouver à côté de l'exécutable (pas de RPATH) : on la copie
# après chaque build. Sous Linux, CMake met le chemin de la .so dans le RPATH de build.
function(fieldnav_copy_mujoco_runtime target)
    if(WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:mujoco::mujoco> $<TARGET_FILE_DIR:${target}>
        )
    endif()
endfunction()
