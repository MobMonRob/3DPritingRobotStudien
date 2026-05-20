#include "validator.h"

#include <cmath>
#include <format>

namespace medusa::kronos {

Validator::Validator(std::shared_ptr<IReachabilityCheck> reach_check)
    : reach_check_{std::move(reach_check)} {}

ValidationResult Validator::validate(const Toolpath& toolpath) const {
    ValidationResult result;
    const auto& meta = toolpath.metadata;
    const auto& wps  = toolpath.waypoints;

    // --- Metadata ---
    if (meta.slicer_version.empty()) {
        result.add("metadata.slicer_version", "Must not be empty.");
    }
    if (meta.reference_frame.empty()) {
        result.add("metadata.reference_frame", "Must not be empty.");
    }

    // --- At least one waypoint ---
    if (wps.empty()) {
        result.add("waypoints", "Toolpath contains no waypoints (at least 1 required).");
        return result; // early exit: further checks would be meaningless
    }

    // --- Validate each waypoint ---
    std::uint32_t last_layer_id = 0;

    for (std::size_t i = 0; i < wps.size(); ++i) {
        const auto& wp     = wps[i];
        const auto  prefix = std::format("waypoints[{}]", i);

        // Finite position values
        if (!std::isfinite(wp.x)) result.add(prefix + ".x", "Non-finite value.");
        if (!std::isfinite(wp.y)) result.add(prefix + ".y", "Non-finite value.");
        if (!std::isfinite(wp.z)) result.add(prefix + ".z", "Non-finite value.");

        // Finite quaternion components
        if (!std::isfinite(wp.qw)) result.add(prefix + ".qw", "Non-finite value.");
        if (!std::isfinite(wp.qx)) result.add(prefix + ".qx", "Non-finite value.");
        if (!std::isfinite(wp.qy)) result.add(prefix + ".qy", "Non-finite value.");
        if (!std::isfinite(wp.qz)) result.add(prefix + ".qz", "Non-finite value.");

        // Quaternion norm ~= 1  (tolerance 1e-6)
        const double norm_sq = wp.qw * wp.qw + wp.qx * wp.qx
                             + wp.qy * wp.qy + wp.qz * wp.qz;
        constexpr double k_tol = 1e-6;
        if (std::abs(norm_sq - 1.0) > k_tol) {
            result.add(prefix + ".q*",
                       std::format("Quaternion norm^2 = {:.9f}, expected 1.0 +/-{}", norm_sq, k_tol));
        }

        // Finite process parameters
        if (!std::isfinite(wp.feed_rate)) {
            result.add(prefix + ".feed_rate", "Non-finite value.");
        }
        if (!std::isfinite(wp.extrusion)) {
            result.add(prefix + ".extrusion", "Non-finite value.");
        }

        // Monotonically non-decreasing layer IDs.
        // Multiple waypoints may share the same layer, so equal consecutive IDs are allowed.
        if (i > 0 && wp.layer_id < last_layer_id) {
            result.add(prefix + ".layer_id",
                       std::format("Layer ID {} is less than previous ID {}.",
                                   wp.layer_id, last_layer_id));
        }
        last_layer_id = wp.layer_id;

        // Reachability check (pluggable interface)
        auto reach_result = reach_check_->check(wp);
        for (auto& err : reach_result.errors) {
            result.add(prefix + "." + err.field_path, err.message);
        }
    }

    return result;
}

} // namespace medusa::kronos
