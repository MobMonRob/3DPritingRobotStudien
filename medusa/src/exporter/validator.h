#pragma once

#include "toolpath.h"
#include "waypoint.h"
#include <memory>
#include <string>
#include <vector>

namespace medusa::kronos {

/// A single structured validation error with field path and message.
struct ValidationError {
    std::string field_path; ///< Path to the offending field, e.g. "waypoints[3].qw"
    std::string message;    ///< Human-readable error description
};

/// Result of a validation run: empty means no errors.
struct ValidationResult {
    std::vector<ValidationError> errors;

    [[nodiscard]] bool is_valid() const noexcept { return errors.empty(); }

    /// Appends a validation error.
    void add(std::string field_path, std::string message) {
        errors.push_back({std::move(field_path), std::move(message)});
    }
};

// ---------------------------------------------------------------------------
// Reachability hook (Strategy pattern)
// ---------------------------------------------------------------------------

/// Abstract interface for UR3e reachability checks.
/// The default implementation (DefaultReachabilityCheck) accepts every waypoint.
/// A real UR3e check can be supplied as a separate implementation.
class IReachabilityCheck {
public:
    virtual ~IReachabilityCheck() = default;

    /// Checks whether the robot can reach the given waypoint.
    /// @return ValidationResult with errors if unreachable, empty otherwise.
    [[nodiscard]] virtual ValidationResult check(const Waypoint& wp) const = 0;
};

/// Default implementation: accepts every waypoint as reachable.
class DefaultReachabilityCheck final : public IReachabilityCheck {
public:
    [[nodiscard]] ValidationResult check(const Waypoint& /*wp*/) const override {
        return {}; // always ok
    }
};

// ---------------------------------------------------------------------------
// Validator
// ---------------------------------------------------------------------------

/// Validates a Toolpath before export for consistency and completeness.
class Validator {
public:
    /// Constructor with an optional reachability hook.
    /// Falls back to DefaultReachabilityCheck if none is provided.
    explicit Validator(
        std::shared_ptr<IReachabilityCheck> reach_check =
            std::make_shared<DefaultReachabilityCheck>()
    );

    /// Validates the entire toolpath.
    /// Checks: required fields, finite values, quaternion norm, monotone layer IDs,
    /// at least one waypoint, and reachability of every waypoint.
    [[nodiscard]] ValidationResult validate(const Toolpath& toolpath) const;

private:
    std::shared_ptr<IReachabilityCheck> reach_check_;
};

} // namespace medusa::kronos
