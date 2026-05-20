set(MEDUSA_LOG_CATEGORY "CONTROLS_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(CONTROLS_SOURCES
        # --- Source Files ---
        "camera_controller.cpp"
        "model_controller.cpp"

        # --- Header Files ---
        "camera_controller.h"
        "model_controller.h"
)

# -----------------------------------------------------------------------------
# Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_controls STATIC ${CONTROLS_SOURCES})
add_library(medusa::controls ALIAS medusa_controls)

medusa_log("Target 'medusa::controls' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(medusa_controls PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(medusa_controls PUBLIC
        medusa::core
        medusa::graphics
        ext::glfw
)

medusa_log("Linked deps: medusa::core, ext::glfw" VERBOSE)

# -----------------------------------------------------------------------------
# Compile Definitions
# -----------------------------------------------------------------------------
target_compile_definitions(medusa_controls PUBLIC
        GLFW_INCLUDE_NONE
)
