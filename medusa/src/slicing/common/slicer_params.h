/**
 * @file slicer_params.h
 * @brief Configuration parameters for slicing algorithms.
 */

#ifndef MEDUSA_SRC_SLICING_SLICER_PARAMS_H_
#define MEDUSA_SRC_SLICING_SLICER_PARAMS_H_

namespace slicing
{
    /**
     * @brief Common parameters shared by all slicer algorithms.
     */
    struct SlicerParams
    {
        /// Layer thickness in mm.
        float layer_thickness{0.2f};

        /// Up axis index: 0 = X, 1 = Y, 2 = Z.
        /// Must match the coordinate convention of the loaded mesh.
        /// Default is 1 (Y-up), which is MEDUSA's internal convention.
        int up_axis{1};

        static constexpr float LAYER_THICKNESS_MIN = 0.01f;
        static constexpr float LAYER_THICKNESS_MAX = 5.0f;
    };

    /**
     * @brief Parameters specific to the Base slicer algorithm.
     */
    struct BaseParams : SlicerParams
    {
        /// Maximum overhang angle (degrees from growth direction) before
        /// a virtual wall is created.  0 = no overhang allowed, 89 = very permissive.
        float overhang_angle{45.0f};

        /// Safety limit for the number of queue-based slicing runs.
        int max_iterations{20};

        static constexpr float OVERHANG_ANGLE_MIN = 0.0f;
        static constexpr float OVERHANG_ANGLE_MAX = 89.0f;
        static constexpr int   MAX_ITERATIONS_MIN = 1;
        static constexpr int   MAX_ITERATIONS_MAX = 100;
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_SLICER_PARAMS_H_
