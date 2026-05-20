#pragma once

namespace medusa::kronos {

/// Schema version of this library (SemVer).
/// Embedded in every exported JSON file as the "schema_version" field.
/// The major version MUST be incremented on backwards-incompatible changes.
inline constexpr char k_schema_version[] = "1.0.0";

} // namespace medusa::kronos
