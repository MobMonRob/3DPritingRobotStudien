set(MEDUSA_LOG_CATEGORY "CORE_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(CORE_SOURCES
        # --- Source Files ---
        "camera.cpp"
        "logger.cpp"

        # --- Header Files ---
        "camera.h"
        "logger.h"
)

# -----------------------------------------------------------------------------
# 2. Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_core STATIC ${CORE_SOURCES})
add_library(medusa::core ALIAS medusa_core)

medusa_log("Target 'medusa::core' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
# PUBLIC = Any target linking medusa::core inherits this include path.
target_include_directories(medusa_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(medusa_core PUBLIC
        ext::glm
        ext::spdlog
)

medusa_log("Linked deps: ext::glm, ext::spdlog, ext::assimp, ext::glad, ext::glfw" VERBOSE)

# -----------------------------------------------------------------------------
# Compiler Definitions
# -----------------------------------------------------------------------------
target_compile_definitions(medusa_core PUBLIC
        GLFW_INCLUDE_NONE
        GL_SILENCE_DEPRECATION
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    medusa_log("SPDLOG_ACTIVE_LEVEL=TRACE (Debug)" DEBUG)
    target_compile_definitions(medusa_core PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
else ()
    medusa_log("SPDLOG_ACTIVE_LEVEL=INFO (Release/RelWithDebInfo)" DEBUG)
    target_compile_definitions(medusa_core PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)
endif ()