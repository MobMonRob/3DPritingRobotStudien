set(MEDUSA_LOG_CATEGORY "UNIT_TEST")

file(GLOB UNIT_TEST_SOURCES "*.cpp")

list(LENGTH UNIT_TEST_SOURCES _medusa_unit_test_count)
medusa_log("Target MedusaUnitTests (${_medusa_unit_test_count} sources)")
if (_medusa_unit_test_count EQUAL 0)
    medusa_log("No *.cpp unit tests found" , WARNING)
endif ()

add_executable(MedusaUnitTests ${UNIT_TEST_SOURCES})

target_link_libraries(MedusaUnitTests PRIVATE
        medusa::core
        medusa::graphics
        medusa::geometry
        medusa::slicing
        medusa::controls
        medusa::export
        ext::gtest_main
)

target_compile_features(MedusaUnitTests PRIVATE cxx_std_23)

target_compile_definitions(MedusaUnitTests PRIVATE
        GLFW_INCLUDE_NONE
        GL_SILENCE_DEPRECATION
        TEST_ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../assets"
)

include(GoogleTest)
medusa_log("gtest_discover_tests(MedusaUnitTests)" , VERBOSE)
gtest_discover_tests(MedusaUnitTests)
