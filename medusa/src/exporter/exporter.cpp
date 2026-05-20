#include "exporter.h"
#include "serializer.h"
#include "logger.h"

#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace medusa::kronos {

KronosExporter::KronosExporter(
    std::filesystem::path      output_dir,
    std::shared_ptr<Validator> validator
)
    : output_dir_{std::move(output_dir)}
    , validator_ {std::move(validator)}
{
    MEDUSA_DEBUG("KronosExporter: output_dir = {}", output_dir_.string());
}

Result<std::filesystem::path> KronosExporter::export_job(const Toolpath& toolpath) {
    MEDUSA_INFO("KronosExporter: starting export ({} waypoints, algorithm={})",
                toolpath.waypoints.size(),
                algorithm_tag(toolpath.metadata.algorithm));

    // 1. Validation
    const ValidationResult vr = validator_->validate(toolpath);
    if (!vr.is_valid()) {
        std::ostringstream oss;
        oss << "Toolpath validation failed (" << vr.errors.size() << " error(s)):";
        for (const auto& err : vr.errors) {
            oss << "\n  [" << err.field_path << "] " << err.message;
        }
        MEDUSA_ERROR("KronosExporter: {}", oss.str());
        return Result<std::filesystem::path>::err(oss.str());
    }
    MEDUSA_DEBUG("KronosExporter: validation passed");

    // 2. Serialisation
    nlohmann::json j;
    try {
        j = to_json(toolpath);
    } catch (const std::exception& ex) {
        MEDUSA_ERROR("KronosExporter: serialisation failed: {}", ex.what());
        return Result<std::filesystem::path>::err(
            std::string("Serialisation failed: ") + ex.what()
        );
    }
    MEDUSA_DEBUG("KronosExporter: serialised {} bytes (unformatted)", j.dump().size());

    // 3. Create output directory if it does not exist
    std::error_code ec;
    std::filesystem::create_directories(output_dir_, ec);
    if (ec) {
        MEDUSA_ERROR("KronosExporter: failed to create output directory '{}': {}",
                     output_dir_.string(), ec.message());
        return Result<std::filesystem::path>::err(
            "Failed to create output directory: " + ec.message()
        );
    }

    // 4. Build file name
    const std::string timestamp = make_timestamp();
    const std::string algo_tag  = algorithm_tag(toolpath.metadata.algorithm);
    const std::string filename  = "toolpath_" + timestamp + "_" + algo_tag + ".json";

    const std::filesystem::path final_path = output_dir_ / filename;
    const std::filesystem::path tmp_path   = std::filesystem::path{final_path} += ".tmp";

    MEDUSA_DEBUG("KronosExporter: writing to tmp file '{}'", tmp_path.string());

    // 5. Atomic write: write to .json.tmp first, then rename
    {
        std::ofstream ofs{tmp_path};
        if (!ofs) {
            MEDUSA_ERROR("KronosExporter: cannot open temporary file '{}'", tmp_path.string());
            return Result<std::filesystem::path>::err(
                "Cannot open temporary file: " + tmp_path.string()
            );
        }
        ofs << j.dump(2); // indented for readability
        if (!ofs) {
            MEDUSA_ERROR("KronosExporter: write failed for '{}'", tmp_path.string());
            return Result<std::filesystem::path>::err(
                "Write to temporary file failed: " + tmp_path.string()
            );
        }
    } // ofs closed here (RAII)

    std::filesystem::rename(tmp_path, final_path, ec);
    if (ec) {
        MEDUSA_ERROR("KronosExporter: atomic rename '{}' -> '{}' failed: {}",
                     tmp_path.string(), final_path.string(), ec.message());
        // Clean up: remove the temporary file
        std::filesystem::remove(tmp_path, ec);
        return Result<std::filesystem::path>::err(
            "Atomic rename failed: " + ec.message()
        );
    }

    MEDUSA_INFO("KronosExporter: export successful -> '{}'", final_path.string());
    return Result<std::filesystem::path>::ok(final_path);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

std::string KronosExporter::make_timestamp() noexcept {
    // UTC timestamp, filesystem-friendly (colons replaced by hyphens).
    // Format: "2026-05-08T14-30-00Z" = 20 chars + null terminator → buf needs 21 bytes.
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm utc_tm{};
#if defined(_WIN32)
    gmtime_s(&utc_tm, &t);
#else
    gmtime_r(&t, &utc_tm);
#endif

    char buf[21]{};  // 20 chars + null terminator
    if (std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H-%M-%SZ", &utc_tm) == 0) {
        return "0000-00-00T00-00-00Z"; // fallback: strftime failed (should never happen)
    }
    return buf;
}

std::string KronosExporter::algorithm_tag(SlicerAlgorithm algo) noexcept {
    switch (algo) {
        case SlicerAlgorithm::Planar:    return "planar";
        case SlicerAlgorithm::Base:      return "base";
        case SlicerAlgorithm::Sphere:    return "sphere";
        case SlicerAlgorithm::Crystal: return "crystal";
    }
    return "unknown";
}

} // namespace medusa::kronos
