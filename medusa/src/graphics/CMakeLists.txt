set(MEDUSA_LOG_CATEGORY "GRAPHICS_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(GRAPHICS_SOURCES
        # --- Source Files ---
        "axes_renderer.cpp"
        "grid_renderer.cpp"
        "mesh.cpp"
        "mesh_loader.cpp"
        "model_transform.cpp"
        "scene_renderer.cpp"
        "shader_program.cpp"
        "toolpath_renderer.cpp"
        "phi_overlay_renderer.cpp"

        # --- Header Files ---
        "axes_renderer.h"
        "grid_renderer.h"
        "mesh.h"
        "mesh_loader.h"
        "model_transform.h"
        "scene_renderer.h"
        "shader_program.h"
        "toolpath_renderer.h"
        "phi_overlay_renderer.h"
)

# -----------------------------------------------------------------------------
# Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_graphics STATIC ${GRAPHICS_SOURCES})
add_library(medusa::graphics ALIAS medusa_graphics)

medusa_log("Target 'medusa::graphics' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(medusa_graphics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(medusa_graphics PUBLIC
        medusa::core
        ext::glad
        ext::glm
        ext::glfw
        ext::assimp
)

# Include slicing headers for ToolpathRenderer (no link needed, only data structs used)
target_include_directories(medusa_graphics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../slicing)

# Phi overlay renderer needs the geometry::TriangleMesh definition (data only,
# no link required — geometry is header-only structs from medusa_graphics's POV).
target_link_libraries(medusa_graphics PUBLIC medusa::geometry)

medusa_log("Linked deps: medusa::core, ext::glad, ext::glm, ext::glfw, ext::assimp" VERBOSE)

# -----------------------------------------------------------------------------
# Compile Definitions
# -----------------------------------------------------------------------------
target_compile_definitions(medusa_graphics PUBLIC
        GLFW_INCLUDE_NONE
        GL_SILENCE_DEPRECATION
        MEDUSA_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)