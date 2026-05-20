set(MEDUSA_LOG_CATEGORY "SLICING_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(SLICING_SOURCES
        # --- Source Files ---
        exporter/Placeholder.cpp

        # --- Header Files ---
        exporter/Placeholder.h
)

# -----------------------------------------------------------------------------
# Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_slicing STATIC ${SLICING_SOURCES})
add_library(medusa::slicing ALIAS medusa_slicing)

medusa_log("Target 'medusa::slicing' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(medusa_slicing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(medusa_slicing PUBLIC
        medusa::core
)

medusa_log("Linked deps: medusa::core" VERBOSE)

# -----------------------------------------------------------------------------
# Compile Definitions
# -----------------------------------------------------------------------------
target_compile_definitions(medusa_slicing PUBLIC
        GLFW_INCLUDE_NONE
        GL_SILENCE_DEPRECATION
)