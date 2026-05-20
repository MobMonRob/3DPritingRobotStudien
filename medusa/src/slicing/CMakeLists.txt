set(MEDUSA_LOG_CATEGORY "SLICING_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(SLICING_SOURCES
        # --- Common Components ---
        "common/toolpath.cpp"
        "common/infill_generator.cpp"
        "common/path_planner.cpp"
        "common/slicing_pipeline.cpp"
        "common/layer.h"
        "common/segment.h"
        "common/toolpath.h"
        "common/slicer_params.h"
        "common/i_slicer.h"
        "common/infill_generator.h"
        "common/path_planner.h"
        "common/slicing_pipeline.h"

        # --- Planar Slicer ---
        "planar/planar_slicer.cpp"
        "planar/planar_slicer.h"

        # --- Base Slicer ---
        "base/base_slicer.cpp"
        "base/base_slicer.h"

        # --- Crystal Slicer ---
        "crystal/crystal_slicer.cpp"
        "crystal/alternating_scheduler.cpp"
        "crystal/scalar_field.cpp"
        "crystal/iso_extractor.cpp"
        "crystal/mesh_refiner.cpp"
        "crystal/branch_tracker.cpp"
        "crystal/crystal_infill.cpp"
        "crystal/crystal_params.h"
        "crystal/crystal_slicer.h"
        "crystal/i_branch_scheduler.h"
        "crystal/alternating_scheduler.h"
        "crystal/scalar_field.h"
        "crystal/iso_extractor.h"
        "crystal/mesh_refiner.h"
        "crystal/branch_tracker.h"
        "crystal/crystal_infill.h"
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
        medusa::geometry
)

# crystal-only dependencies — kept PRIVATE so the public API stays light
# and downstream targets (graphics, app) don't pull in Eigen/libigl headers.
target_link_libraries(medusa_slicing PRIVATE
        ext::eigen
        ext::libigl
)

medusa_log("Linked deps: medusa::core, medusa::geometry (PUBLIC); ext::eigen, ext::libigl (PRIVATE)" VERBOSE)

# -----------------------------------------------------------------------------
# Compile Definitions
# -----------------------------------------------------------------------------
target_compile_definitions(medusa_slicing PUBLIC
        GLFW_INCLUDE_NONE
        GL_SILENCE_DEPRECATION
)