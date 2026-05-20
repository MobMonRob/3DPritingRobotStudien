set(MEDUSA_LOG_CATEGORY "UI_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(UI_SOURCES
        # --- Source Files ---
        "im_gui_layer.cpp"
        "file_browser.cpp"
        "ui_renderer.cpp"
        "ui_style.cpp"

        # --- Header Files ---
        "im_gui_layer.h"
        "file_browser.h"
        "ui_renderer.h"
        "ui_style.h"
)

# -----------------------------------------------------------------------------
# Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_ui STATIC ${UI_SOURCES})
add_library(medusa::ui ALIAS medusa_ui)

medusa_log("Target 'medusa::ui' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(medusa_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking (Dependency Management)
# -----------------------------------------------------------------------------
target_link_libraries(medusa_ui PUBLIC
        medusa::graphics
        ext::imgui
)

target_compile_definitions(medusa_ui PRIVATE
        MEDUSA_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)

medusa_log("Linked deps: medusa::graphics, ext::imgui" VERBOSE)