#pragma once

#include "toolpath.h"
#include <nlohmann/json.hpp>

namespace medusa::kronos {

/// Serialises a Toolpath into a nlohmann::json object.
/// - Sets schema_version from version.h automatically.
/// - Computes and sets waypoints_hash (FNV-1a-64) automatically.
/// - Throws std::runtime_error for invalid inputs (NaN, Inf).
[[nodiscard]] nlohmann::json to_json(const Toolpath& toolpath);

/// Deserialises a Toolpath from a nlohmann::json object.
/// - Checks schema_version for major-version compatibility.
/// - Throws std::runtime_error for missing/incompatible schema or missing required fields.
Toolpath from_json(const nlohmann::json& j);

} // namespace medusa::kronos
