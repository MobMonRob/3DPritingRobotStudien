/**
 * @file scalar_field.h
 * @brief Harmonic scalar field Phi over the input mesh (Crystal step 2).
 *
 * Crystal drives its non-planar layers by iso-surfaces of a scalar field
 * Phi : Mesh -> R that grows monotonically from the bottom (build plate) to
 * the top (free surface) of the part.
 *
 * Method choice — harmonic field (delta Phi = 0 with Dirichlet boundary
 * conditions) over heat-method or geodesic distance:
 *   - Maximum principle guarantees no interior extrema, so iso-surfaces stay
 *     simply connected and never form closed pockets inside the volume.
 *   - C-infinity smooth in the interior -> curvature-continuous layers by
 *     construction (the brief explicitly requires "weiche, kruemmungsstetige"
 *     iso-surfaces).
 *   - Single sparse linear solve using libigl's cotangent Laplacian and
 *     min_quad_with_fixed; no diffusion-time tuning, no extra meshing.
 *
 * For the initial step we use the simplest sensible boundary conditions:
 *   Phi = 0 on vertices whose up-axis coordinate is within
 *           bottom_band_fraction of the mesh height of the bottom,
 *   Phi = 1 on vertices within top_band_fraction of the top.
 *
 * Step 3 will replace the Phi=0 set with the actual top boundary of the
 * planar base layers; the rest of the pipeline stays unchanged.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_SCALAR_FIELD_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_SCALAR_FIELD_H_

#include <vector>

#include <glm/vec3.hpp>

namespace geometry { struct TriangleMesh; }

namespace slicing::crystal
{
    /**
     * @brief Result of one harmonic-field solve.
     */
    struct ScalarField
    {
        /// Per-vertex Phi value, length == mesh.numVertices(). Empty on failure.
        std::vector<float> phi;

        /// Per-face gradient magnitude |grad Phi|, length == mesh.numFaces().
        /// Units: 1/mm (Phi is dimensionless, vertex coordinates are in mm).
        /// Used by IsoExtractor for adaptive layer thickness.
        std::vector<float> grad_norm_face;

        /// Per-face gradient vector grad Phi (3D, in mesh coordinates),
        /// length == mesh.numFaces(). Used by IsoExtractor + PoseBuilder to
        /// derive the 6-DOF tool orientation (z_tool = -normalize(grad Phi)).
        std::vector<glm::vec3> grad_face;

        /// Min/max of phi (== 0 and 1 on success unless the solver was bypassed).
        float phi_min{0.0f};
        float phi_max{1.0f};

        /// Statistics on |grad Phi| across faces. grad_mean is the
        /// area-unweighted average and is only used for sanity logging /
        /// adaptive-step bounds.
        float grad_min{0.0f};
        float grad_max{0.0f};
        float grad_mean{0.0f};

        /// True if the linear solve succeeded.
        bool valid{false};
    };

    /**
     * @brief Configuration for the harmonic-field solver.
     */
    struct ScalarFieldConfig
    {
        /// Up axis of the build platform: 0 = X, 1 = Y, 2 = Z.
        int up_axis{1};

        /// Vertices within this fraction of the mesh height above the bottom
        /// are clamped to Phi = 0 (Dirichlet "anchor" boundary).
        /// 0.05 picks roughly the bottom 5%; tune for noisy meshes.
        float bottom_band_fraction{0.02f};

        /// Vertices within this fraction below the top are clamped to Phi = 1.
        float top_band_fraction{0.02f};
    };

    /**
     * @brief Solves the harmonic equation delta Phi = 0 on the mesh vertices
     *        with Dirichlet boundary conditions derived from the up-axis bands.
     *
     * Uses libigl's cotangent Laplacian + min_quad_with_fixed under the hood.
     *
     * @param mesh   Input triangle mesh (must be valid, manifold preferred).
     * @param config Boundary configuration.
     * @return ScalarField with per-vertex Phi values; valid==false on failure.
     */
    [[nodiscard]] ScalarField solveHarmonicField(const geometry::TriangleMesh& mesh,
                                                 const ScalarFieldConfig& config = {});
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_SCALAR_FIELD_H_
