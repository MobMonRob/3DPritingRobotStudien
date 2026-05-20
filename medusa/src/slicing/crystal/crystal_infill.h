/**
 * @file crystal_infill.h
 * @brief Rectilinear infill generator for non-planar (curvilinear) iso-layers.
 *
 * The stock @ref slicing::InfillGenerator assumes flat layers with a constant
 * coordinate along the up-axis. Crystal iso-layers do not satisfy this
 * assumption: every contour point sits on a curved iso-surface of the harmonic
 * field Phi.
 *
 * Algorithm per layer-component:
 *   1. Centroid c and average normal n_hat from the per-vertex orientations
 *      (already populated by the iso-extractor as -normalize(grad Phi)).
 *   2. Tangent basis (u_hat, v_hat) via PCA of the projected boundary so the
 *      sweep direction aligns with the contour's principal axis (cube edges,
 *      crossbar length). Per-layer 90-degree rotation gives cross-hatch.
 *   3. Project the chained boundary polygon to 2D (u, v); even-odd scanline
 *      sweep along v with line spacing @ref spacing.
 *   4. Lift each 2D scanline endpoint back to the curved iso-surface via a
 *      ray-cast against the source mesh along +/- n_hat. Points whose ray
 *      misses the mesh are dropped.
 *   5. Emit infill segments as a connected zigzag (snake pattern): consecutive
 *      scanlines share endpoints so the toolpath builder does not insert
 *      travel moves between them.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_INFILL_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_INFILL_H_

#include <vector>

#include "common/layer.h"

namespace geometry { struct TriangleMesh; }

namespace slicing::crystal
{
    /**
     * @brief Append rectilinear infill segments to each iso-layer in-place.
     *
     * @param layers   Iso-layers (each layer == one connected component).
     * @param spacing  Distance between scan-lines in mm. Must be > 0.
     * @param mesh     The (refined) source mesh used to lift 2D infill points
     *                 back onto the curved iso-surface via ray-cast.
     */
    void generateNonPlanarInfill(std::vector<Layer>& layers,
                                 float spacing,
                                 const geometry::TriangleMesh& mesh);
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_INFILL_H_
