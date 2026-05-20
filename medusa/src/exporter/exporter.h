#pragma once

#include "result.h"
#include "toolpath.h"
#include "validator.h"
#include <filesystem>
#include <memory>

namespace medusa::kronos {

/// Exports a validated Toolpath as a JSON file into a target directory.
///
/// Atomic write: the file is first written as "<name>.json.tmp" and then
/// renamed to "<name>.json". Kronos may safely ignore ".tmp" files.
///
/// Filename format: toolpath_<UTC-ISO-timestamp>_<algorithm>.json
/// Example: toolpath_2026-05-08T14-30-00Z_base.json
class KronosExporter {
public:
    /// Constructor.
    /// @param output_dir  Target directory for JSON export files.
    ///                    Created automatically if it does not exist.
    /// @param validator   Optional validator with a custom reachability hook.
    explicit KronosExporter(
        std::filesystem::path output_dir,
        std::shared_ptr<Validator> validator = std::make_shared<Validator>()
    );

    /// Exports the toolpath.
    /// 1. Validates the toolpath.
    /// 2. Serialises it as JSON (including hash computation).
    /// 3. Writes atomically into the target directory.
    ///
    /// @return Path to the produced .json file, or an error description.
    [[nodiscard]] Result<std::filesystem::path> export_job(const Toolpath& toolpath);

private:
    std::filesystem::path      output_dir_;
    std::shared_ptr<Validator> validator_;

    /// Produces the filesystem-friendly timestamp portion of the filename.
    static std::string make_timestamp() noexcept;

    /// Maps the algorithm enum to a short tag for use in the filename.
    static std::string algorithm_tag(SlicerAlgorithm algo) noexcept;
};

} // namespace medusa::kronos
