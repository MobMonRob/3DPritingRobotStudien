#pragma once

#include <string_view>

namespace medusa::kronos {

/// Motion type of a waypoint.
/// Travel = repositioning move (no extrusion), Print = deposition move.
enum class MotionType {
    Travel, ///< Repositioning move without material deposition
    Print   ///< Deposition move with material extrusion
};

/// Returns the JSON string identifier for the given MotionType.
[[nodiscard]] inline constexpr std::string_view motion_type_to_string(MotionType mt) noexcept {
    switch (mt) {
        case MotionType::Travel: return "travel";
        case MotionType::Print:  return "print";
    }
    return "unknown"; // unreachable
}

} // namespace medusa::kronos
