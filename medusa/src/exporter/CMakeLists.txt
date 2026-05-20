set(MEDUSA_LOG_CATEGORY "EXPORT_LIB")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
set(EXPORT_SOURCES
        # --- Source Files ---
        "serializer.cpp"
        "validator.cpp"
        "exporter.cpp"

        # --- Header Files ---
        "version.h"
        "result.h"
        "motion_type.h"
        "waypoint.h"
        "job_metadata.h"
        "toolpath.h"
        "serializer.h"
        "validator.h"
        "exporter.h"
)

# -----------------------------------------------------------------------------
# Target Setup
# -----------------------------------------------------------------------------
add_library(medusa_export STATIC ${EXPORT_SOURCES})
add_library(medusa::export ALIAS medusa_export)

medusa_log("Target 'medusa::export' configured")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(medusa_export PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Linking
# -----------------------------------------------------------------------------
target_link_libraries(medusa_export PUBLIC ext::json)
target_link_libraries(medusa_export PRIVATE medusa::core)

medusa_log("Linked deps: ext::json, medusa::core" VERBOSE)

# -----------------------------------------------------------------------------
# Compile Features
# -----------------------------------------------------------------------------
target_compile_features(medusa_export PUBLIC cxx_std_20)
