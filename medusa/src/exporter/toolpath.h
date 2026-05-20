#pragma once

#include "job_metadata.h"
#include "waypoint.h"
#include <vector>

namespace medusa::kronos {

/// Complete toolpath: metadata plus an ordered list of waypoints.
struct Toolpath {
    JobMetadata           metadata;  ///< Slicing job metadata
    std::vector<Waypoint> waypoints; ///< Waypoints in execution order
};

} // namespace medusa::kronos
