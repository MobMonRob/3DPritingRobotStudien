#pragma once

#include "version.h"
#include <string>

namespace medusa::kronos {

/// Slicing algorithm that produced this job.
enum class SlicerAlgorithm {
    Planar,    ///< Layer-by-layer planar slicer
    Base,      ///< Non-planar base slicer (primary algorithm)
    Sphere,    ///< Spherical-surface slicer (conceptual)
    Crystal  ///< Curvilinear, scalar-field driven slicer with branch tracking
};

/// Metadata for a slicing job.
/// Embedded alongside the waypoints in the JSON export file.
struct JobMetadata {
    /// MEDUSA slicer version (SemVer, e.g. "0.0.1")
    std::string     slicer_version{};

    /// Algorithm that produced this job
    SlicerAlgorithm algorithm{SlicerAlgorithm::Base};

    /// Position unit – always "mm", but stored explicitly for clarity
    std::string     units_position{"mm"};

    /// Time unit – always "s", but stored explicitly for clarity
    std::string     units_time{"s"};

    /// Reference coordinate frame name (e.g. "workpiece")
    std::string     reference_frame{"workpiece"};

    /// Job creation timestamp (ISO 8601, UTC, e.g. "2026-05-08T14:30:00Z")
    std::string     created_at{};

    /// Schema version of this library – set automatically during export
    std::string     schema_version{k_schema_version};

    /// FNV-1a-64 checksum over the serialised waypoints array (16 hex digits).
    /// Computed automatically during export and verified during import.
    std::string     waypoints_hash{};
};

} // namespace medusa::kronos
