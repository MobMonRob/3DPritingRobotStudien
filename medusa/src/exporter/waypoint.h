#pragma once

#include "motion_type.h"
#include <cstdint>

namespace medusa::kronos {

/// Single 6-DOF waypoint.
/// Position in mm, orientation as a normalised quaternion (w, x, y, z).
struct Waypoint {
    // --- Position in the reference coordinate frame [mm] ---
    double x{0.0}; ///< X coordinate [mm]
    double y{0.0}; ///< Y coordinate [mm]
    double z{0.0}; ///< Z coordinate [mm]

    // --- Orientation as a unit quaternion (|q| = 1) ---
    // Order: w, x, y, z  (ROS convention, compatible with MoveIt / tf2)
    double qw{1.0}; ///< Real part
    double qx{0.0}; ///< Imaginary part i
    double qy{0.0}; ///< Imaginary part j
    double qz{0.0}; ///< Imaginary part k

    // --- Process parameters ---
    double feed_rate{0.0}; ///< Feed rate [mm/s]
    double extrusion{0.0}; ///< Extrusion value [mm^3/s]; 0 for travel moves

    // --- Classification ---
    MotionType    motion_type{MotionType::Travel}; ///< Move type
    std::uint32_t layer_id  {0};                   ///< Layer index (0-based, monotonically non-decreasing)
    std::uint32_t segment_id{0};                   ///< Segment index within a layer
    std::uint32_t branch_id {0};                   ///< Branch identifier (0 = trunk / planar base; 1..N for tracked branches)
};

} // namespace medusa::kronos
