set(MEDUSA_LOG_CATEGORY "INTEGRATION_TEST")

file(GLOB INTEGRATION_TEST_SOURCES "*.cpp")

list(LENGTH INTEGRATION_TEST_SOURCES _medusa_integration_test_count)
medusa_log("Target MedusaIntegrationTests (${_medusa_integration_test_count} sources)")
if (_medusa_integration_test_count EQUAL 0)
    medusa_log("No *.cpp integration tests found" , NOTICE)
endif ()

add_executable(MedusaIntegrationTests ${INTEGRATION_TEST_SOURCES})

target_link_libraries(MedusaIntegrationTests PRIVATE
        medusa::core
        medusa::graphics
        ext::gtest_main
)

target_compile_features(MedusaIntegrationTests PRIVATE cxx_std_23)

target_compile_definitions(MedusaIntegrationTests PRIVATE
        GLFW_INCLUDE_NONE
        GL_SILENCE_DEPRECATION
        TEST_ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../assets"
)

include(GoogleTest)
medusa_log("gtest_discover_tests(MedusaIntegrationTests)" , VERBOSE)
gtest_discover_tests(MedusaIntegrationTests)
