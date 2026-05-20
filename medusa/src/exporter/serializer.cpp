#include "serializer.h"
#include "version.h"
#include "motion_type.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace medusa::kronos {

// ---------------------------------------------------------------------------
// FNV-1a 64-bit hash
// Chosen because no cryptographic library is available.
// FNV-1a is deterministic, low-collision for short strings, and trivial to
// implement. Cryptographic security is not required here – the hash is used
// only for integrity checking.
// ---------------------------------------------------------------------------
namespace {

constexpr std::uint64_t k_fnv1a_offset = 14695981039346656037ULL;
constexpr std::uint64_t k_fnv1a_prime  = 1099511628211ULL;

[[nodiscard]] std::uint64_t fnv1a_64(std::string_view data) noexcept {
    std::uint64_t hash = k_fnv1a_offset;
    for (unsigned char c : data) {
        hash ^= static_cast<std::uint64_t>(c);
        hash *= k_fnv1a_prime;
    }
    return hash;
}

[[nodiscard]] std::string to_hex(std::uint64_t value) {
    constexpr char digits[] = "0123456789abcdef";
    std::string result(16, '0');
    for (int i = 15; i >= 0; --i) {
        result[static_cast<std::size_t>(i)] = digits[value & 0xFu];
        value >>= 4u;
    }
    return result;
}

// Helper: SlicerAlgorithm -> JSON string
[[nodiscard]] std::string_view algorithm_to_string(SlicerAlgorithm algo) {
    switch (algo) {
        case SlicerAlgorithm::Planar:    return "planar";
        case SlicerAlgorithm::Base:      return "base";
        case SlicerAlgorithm::Sphere:    return "sphere";
        case SlicerAlgorithm::Crystal: return "crystal";
    }
    return "unknown";
}

[[nodiscard]] SlicerAlgorithm algorithm_from_string(const std::string& s) {
    if (s == "planar")    return SlicerAlgorithm::Planar;
    if (s == "base")      return SlicerAlgorithm::Base;
    if (s == "sphere")    return SlicerAlgorithm::Sphere;
    if (s == "crystal") return SlicerAlgorithm::Crystal;
    throw std::runtime_error("Unknown algorithm: " + s);
}

// Serialises the waypoint list without the hash field (used for hash computation).
[[nodiscard]] nlohmann::json waypoints_to_json_array(const std::vector<Waypoint>& waypoints) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& wp : waypoints) {
        arr.push_back({
            {"x",          wp.x},
            {"y",          wp.y},
            {"z",          wp.z},
            {"qw",         wp.qw},
            {"qx",         wp.qx},
            {"qy",         wp.qy},
            {"qz",         wp.qz},
            {"feed_rate",  wp.feed_rate},
            {"extrusion",  wp.extrusion},
            {"motion_type",std::string(motion_type_to_string(wp.motion_type))},
            {"layer_id",   wp.layer_id},
            {"segment_id", wp.segment_id},
            {"branch_id",  wp.branch_id}
        });
    }
    return arr;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

nlohmann::json to_json(const Toolpath& toolpath) {
    // Serialise waypoints and compute hash
    nlohmann::json wp_array = waypoints_to_json_array(toolpath.waypoints);
    const std::string wp_str = wp_array.dump();
    const std::string hash   = to_hex(fnv1a_64(wp_str));

    const auto& meta = toolpath.metadata;

    nlohmann::json j;
    j["schema_version"]  = k_schema_version;
    j["slicer_version"]  = meta.slicer_version;
    j["algorithm"]       = std::string(algorithm_to_string(meta.algorithm));
    j["units"]           = {{"position", meta.units_position},
                             {"time",     meta.units_time}};
    j["reference_frame"] = meta.reference_frame;
    j["created_at"]      = meta.created_at;
    j["waypoints_hash"]  = hash;   // FNV-1a-64 over wp_array.dump()
    j["waypoints"]       = std::move(wp_array);

    return j;
}

Toolpath from_json(const nlohmann::json& j) {
    // Schema version: major must match
    if (!j.contains("schema_version")) {
        throw std::runtime_error("Required field 'schema_version' is missing.");
    }
    const std::string sv = j.at("schema_version").get<std::string>();
    const int imported_major = std::stoi(sv.substr(0, sv.find('.')));
    const int current_major  = std::stoi(
        std::string(k_schema_version).substr(0, std::string(k_schema_version).find('.'))
    );
    if (imported_major != current_major) {
        throw std::runtime_error(
            "Incompatible schema_version: imported=" + sv +
            ", expected major=" + std::to_string(current_major)
        );
    }

    Toolpath tp;
    auto& meta = tp.metadata;

    meta.schema_version  = sv;
    meta.slicer_version  = j.value("slicer_version", "");
    meta.algorithm       = algorithm_from_string(j.at("algorithm").get<std::string>());
    meta.reference_frame = j.value("reference_frame", "workpiece");
    meta.created_at      = j.value("created_at", "");
    meta.waypoints_hash  = j.value("waypoints_hash", "");

    if (j.contains("units")) {
        meta.units_position = j.at("units").value("position", "mm");
        meta.units_time     = j.at("units").value("time", "s");
    }

    // Deserialise waypoints
    if (!j.contains("waypoints") || !j.at("waypoints").is_array()) {
        throw std::runtime_error("Required field 'waypoints' is missing or not an array.");
    }

    for (const auto& item : j.at("waypoints")) {
        Waypoint wp;
        wp.x          = item.at("x").get<double>();
        wp.y          = item.at("y").get<double>();
        wp.z          = item.at("z").get<double>();
        wp.qw         = item.at("qw").get<double>();
        wp.qx         = item.at("qx").get<double>();
        wp.qy         = item.at("qy").get<double>();
        wp.qz         = item.at("qz").get<double>();
        wp.feed_rate  = item.at("feed_rate").get<double>();
        wp.extrusion  = item.at("extrusion").get<double>();
        wp.layer_id   = item.at("layer_id").get<std::uint32_t>();
        wp.segment_id = item.at("segment_id").get<std::uint32_t>();
        wp.branch_id  = item.value("branch_id", std::uint32_t{0});

        const std::string mt = item.at("motion_type").get<std::string>();
        if (mt == "travel")      wp.motion_type = MotionType::Travel;
        else if (mt == "print")  wp.motion_type = MotionType::Print;
        else throw std::runtime_error("Unknown motion_type: " + mt);

        tp.waypoints.push_back(wp);
    }

    return tp;
}

} // namespace medusa::kronos
