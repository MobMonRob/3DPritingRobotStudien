/**
 * @file crystal_params.h
 * @brief Configuration parameters for the Crystal non-planar slicer.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_PARAMS_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_PARAMS_H_

#include <memory>

#include "common/slicer_params.h"

namespace slicing::crystal
{
    class IBranchScheduler; // forward decl

    /**
     * @brief Parameters specific to the Crystal algorithm.
     *
     * Crystal produces curvilinear print layers that follow the geometry's
     * natural growth direction (iso-surfaces of a harmonic scalar field Phi).
     * The first @ref planar_base_layers layers are kept planar for bed adhesion.
     */
    struct CrystalParams : SlicerParams
    {
        /// Number of initial planar layers for bed adhesion. Default 4.
        int planar_base_layers{4};

        /// Iso-layer thickness (delta in Phi units, scaled to mm). Default = layer_thickness.
        /// If <= 0, falls back to SlicerParams::layer_thickness.
        float iso_layer_thickness{0.0f};

        /// Number of perimeter (wall) loops per layer component.
        /// Currently fixed at 1 - not exposed in UI.
        /// TODO: Multi-wall support for curved layers requires geodesic offset implementation.
        int wall_count{1};

        /// Spacing between rectilinear infill scan-lines (mm).
        float infill_spacing{1.5f};

        /// Tool-tilt warning threshold (degrees from build platform up axis).
        /// Not exposed in UI - overhang safety is not applicable to non-planar slicing;
        /// layers follow geometry rather than creating traditional overhangs.
        /// Kept as internal parameter for potential future diagnostic logging.
        float overhang_safety_deg{60.0f};

        /// If true, locally adapt layer thickness to gradient magnitude.
        /// Phase-2 feature; ignored by the initial implementation.
        bool adaptive_thickness{false};

        /// Optional injected branch-scheduling strategy.
        /// If null, the CrystalSlicer constructs an AlternatingScheduler by default.
        /// shared_ptr so the params struct stays copyable.
        std::shared_ptr<IBranchScheduler> scheduler{};

        /// Maximum edge length (mm) the internal Crystal mesh refiner
        /// will leave in the working copy of the mesh before the harmonic
        /// Phi-solve. Coarse inputs (e.g. a 16-vertex T) otherwise give
        /// the solver too few degrees of freedom and Phi degenerates into
        /// a trivial linear ramp. Set to <= 0 to disable refinement.
        float refine_max_edge_length{1.0f};

        // -------- Bounds for UI sliders --------
        static constexpr int   PLANAR_BASE_LAYERS_MIN = 0;
        static constexpr int   PLANAR_BASE_LAYERS_MAX = 50;
        static constexpr float INFILL_SPACING_MIN     = 0.1f;
        static constexpr float INFILL_SPACING_MAX     = 20.0f;
    };
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_PARAMS_H_
