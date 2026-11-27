# Tests du module path (GoogleTest) + vérification que les scènes générées sont à jour.

add_executable(test_path
    test_reference.cpp
)
target_link_libraries(test_path PRIVATE fieldnav::path GTest::gtest_main)
target_compile_definitions(test_path PRIVATE
    FIELDNAV_DATA_DIR="${PROJECT_SOURCE_DIR}/data"
)
fieldnav_enable_warnings(test_path)
fieldnav_enable_sanitizers(test_path)

add_test(NAME path.unit COMMAND test_path)

# Les fichiers commités (scenes/row_straight.xml, data/reference_straight.csv) doivent être
# exactement ceux que produit scripts/gen_row_scene.py : sinon, quelqu'un les a modifiés à la
# main ou a changé le script sans régénérer.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME path.scene_up_to_date
        COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/gen_row_scene.py --check
    )
else()
    message(STATUS "Python 3 introuvable : test path.scene_up_to_date désactivé")
endif()
