/**
 * @file infill_generator.h
 * @brief Generates 100% infill paths within layer contours.
 */

#ifndef MEDUSA_SRC_SLICING_INFILL_GENERATOR_H_
#define MEDUSA_SRC_SLICING_INFILL_GENERATOR_H_

#include <vector>
#include "layer.h"

namespace slicing
{
    /**
     * @brief Generates infill paths for a set of layers.
     *
     * Currently implements 100% infill with non-planar zigzag lines
     * that follow the layer surface curvature.
     */
    class InfillGenerator
    {
    public:
        /**
         * @brief Generates infill points within each layer.
         *
         * Adds infill points to the layer's contour data. The infill follows
         * the layer surface orientation (non-planar capable).
         *
         * @param layers Layers to fill (modified in place).
         * @param lineSpacing Spacing between infill lines in mm (default = layer thickness).
         */
        static void generate(std::vector<Layer>& layers, float lineSpacing = 0.0f);
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_INFILL_GENERATOR_H_
