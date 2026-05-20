set(MEDUSA_LOG_CATEGORY "GEOMETRY_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(GEOMETRY_SOURCES
        # --- Source Files ---
        "mesh_converter.cpp"

        # --- Header Files ---
        "aabb.h"
        "triangle_mesh.h"
        "mesh_converter.h"
)

# -----------------------------------------------------------------------------
# Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_geometry STATIC ${GEOMETRY_SOURCES})
add_library(medusa::geometry ALIAS medusa_geometry)

medusa_log("Target 'medusa::geometry' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(medusa_geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(medusa_geometry PUBLIC
        medusa::core
)

medusa_log("Linked deps: medusa::core" VERBOSE)
