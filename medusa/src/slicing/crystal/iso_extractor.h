/**
 * @file iso_extractor.h
 * @brief Extracts non-planar iso-contour layers from a scalar field Phi.
 *
 * Step 3 of Crystal. For each prescribed iso-value phi*, walk every face
 * of the mesh, intersect the plane phi == phi* with the triangle in
 * (vertex,phi)-space (linear interpolation along edges), then chain the
 * resulting 3D segments into closed contour rings. Each ring becomes a
 * non-planar @ref slicing::Layer.
 *
 * Notes:
 *  - Iso-values are spaced uniformly in phi-space. Mapping a desired
 *    layer thickness in mm to a delta-phi is approximated by
 *    dh / mesh_height (constant-thickness assumption — adaptive layer
 *    thickness based on |grad phi| is a phase-2 feature).
 *  - Per-vertex normals are filled with the up-axis unit vector for now;
 *    Step 6 (PoseBuilder) replaces them with -grad phi to drive the 6-DOF
 *    tool orientation.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_ISO_EXTRACTOR_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_ISO_EXTRACTOR_H_

#include <cstdint>
#include <vector>

#include "common/layer.h"
#include "scalar_field.h"

namespace geometry { struct TriangleMesh; }

namespace slicing::crystal
{
    struct IsoExtractorConfig
    {
        /// Up axis (0=X, 1=Y, 2=Z). Used only for fallback normals + logging.
        int up_axis{1};

        /// Desired iso-layer thickness in mm (physical distance between
        /// adjacent iso-surfaces, NOT a delta-phi). Must be > 0.
        ///
        /// In adaptive mode the iso-step is locally scaled by |grad Phi|
        /// so that the mean physical spacing matches this value:
        ///     dphi_local = layer_thickness * |grad Phi|_local
        /// In uniform mode the constant-thickness assumption
        ///     dphi = layer_thickness / mesh_height * (phi_max - phi_min)
        /// is used (valid only for cylindrical / single-stem geometries).
        float layer_thickness{0.4f};

        /// Inclusive-exclusive iso-value range. Iso-values are sampled in
        /// (phi_start, phi_end) — the bounds themselves are skipped because
        /// extracting at the Dirichlet boundary produces degenerate contours.
        float phi_start{0.0f};
        float phi_end{1.0f};

        /// Hard cap on the number of iso-layers (safety against pathological
        /// configurations). 2000 is generous for any realistic FFF print.
        int max_layers{2000};

        /// If true, dphi is locally adapted to |grad Phi| so the physical
        /// spacing between iso-surfaces stays close to @ref layer_thickness.
        /// Strongly recommended for any non-cylindrical geometry (T-shapes,
        /// tapers, branches). Defaults to true.
        bool adaptive{true};

        /// Safety bounds on the adaptive dphi step, expressed as multipliers
        /// of the global mean dphi. Prevent runaway steps in regions of
        /// near-zero / extreme gradient (e.g. pinch points in a double pyramid).
        float dphi_min_factor{0.05f};
        float dphi_max_factor{8.0f};
    };

    /**
     * @brief Extract non-planar iso-layers from a harmonic scalar field.
     *
     * @param mesh                 Input triangle mesh (must match @p field).
     * @param field                Per-vertex scalar field (length == numVertices()).
     * @param cfg                  Extraction configuration.
     * @param starting_layer_index Index assigned to the first emitted layer
     *                             (continues numbering after the planar base).
     * @return Ordered iso-layers (low phi -> high phi). Empty on failure.
     */
    [[nodiscard]] std::vector<Layer>
    extractIsoLayers(const geometry::TriangleMesh& mesh,
                     const ScalarField& field,
                     const IsoExtractorConfig& cfg,
                     std::uint32_t starting_layer_index);
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_ISO_EXTRACTOR_H_
