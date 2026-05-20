set(MEDUSA_LOG_CATEGORY "APP")

# -----------------------------------------------------------------------------
# Sources explicit
# -----------------------------------------------------------------------------
set(APP_SOURCES
        # --- Source Files ---
        "main.cpp"
        "app.cpp"

        # --- Header Files ---
        "app.h"
)

# -----------------------------------------------------------------------------
# Executable Creation
# -----------------------------------------------------------------------------
medusa_log("Creating executable target: MedusaApp")
add_executable(MedusaApp ${APP_SOURCES})

target_compile_features(MedusaApp PRIVATE cxx_std_23)
medusa_log("Enforcing C++ Standard: C++23" VERBOSE)

target_compile_definitions(MedusaApp PRIVATE
        GLFW_INCLUDE_NONE
        MEDUSA_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(MedusaApp PRIVATE
        medusa::ui
        medusa::graphics
        medusa::controls
        medusa::core
)
medusa_log("Linked internal modules: medusa::ui, medusa::graphics, medusa::core" VERBOSE)

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

if (APPLE)
    medusa_log("Platform: macOS detected")
    medusa_log("Configuring as macOS Bundle (.app)...")

    set_target_properties(MedusaApp PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_BUNDLE_NAME "Medusa Engine"
            MACOSX_BUNDLE_GUI_IDENTIFIER "com.axtmann.medusa"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
            MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
            MACOSX_BUNDLE_COPYRIGHT "Copyright (c) 2026 Medusa Team"
            OUTPUT_NAME "Medusa"
    )
    medusa_log("  Bundle ID: com.axtmann.medusa" VERBOSE)
    medusa_log("  Version:   ${PROJECT_VERSION}" VERBOSE)

    find_library(COCOA_LIB Cocoa)
    find_library(IOKIT_LIB IOKit)
    find_library(COREFOUNDATION_LIB CoreFoundation)
    find_library(OPENGL_LIB OpenGL)

    if (COCOA_LIB AND IOKIT_LIB AND COREFOUNDATION_LIB AND OPENGL_LIB)
        target_link_libraries(MedusaApp PRIVATE
                ${COCOA_LIB}
                ${IOKIT_LIB}
                ${COREFOUNDATION_LIB}
                ${OPENGL_LIB}
        )
        medusa_log("  Linked System Frameworks: Cocoa, IOKit, CoreFoundation, OpenGL" VERBOSE)
    else ()
        medusa_log("  Error: Required macOS frameworks not found!" FATAL_ERROR)
    endif ()

    target_compile_definitions(MedusaApp PRIVATE GL_SILENCE_DEPRECATION)

elseif (UNIX AND NOT APPLE)
    medusa_log("Platform: UNIX (Linux) detected")

    set_target_properties(MedusaApp PROPERTIES
            OUTPUT_NAME "Medusa-${PROJECT_VERSION}"
    )
    medusa_log("  Output Binary: Medusa-${PROJECT_VERSION}" VERBOSE)

    find_package(X11 REQUIRED)
    if (X11_FOUND)
        target_link_libraries(MedusaApp PRIVATE X11::X11)
        medusa_log("  Linked X11 library" VERBOSE)
    endif ()

elseif (WIN32)
    medusa_log("Platform: Windows detected")

    set_target_properties(MedusaApp PROPERTIES
            OUTPUT_NAME "Medusa-${PROJECT_VERSION}"
    )
    medusa_log("  Output Binary: Medusa-${PROJECT_VERSION}.exe" VERBOSE)

    target_compile_definitions(MedusaApp PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    medusa_log("  Applied definitions: WIN32_LEAN_AND_MEAN, NOMINMAX" VERBOSE)
endif ()

medusa_log("Target 'MedusaApp' configured successfully")