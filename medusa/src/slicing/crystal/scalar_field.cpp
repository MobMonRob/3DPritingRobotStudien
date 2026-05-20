/**
 * @file scalar_field.cpp
 * @brief Harmonic scalar field implementation using libigl + Eigen.
 */

#include "scalar_field.h"

#include <algorithm>
#include <array>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Sparse>

// Eigen 5.x relocated `all` into Eigen::placeholders::; libigl 2.5 still
// references `Eigen::all` directly in its templated headers. Re-export the
// symbol at the namespace level for this translation unit so libigl compiles
// against the newer Eigen. Scoped to this .cpp; no global side effects.
namespace Eigen { using placeholders::all; }

#include <igl/cotmatrix.h>
#include <igl/grad.h>
#include <igl/min_quad_with_fixed.h>

#include "logger.h"
#include "triangle_mesh.h"

namespace slicing::crystal
{
    namespace
    {
        /// Converts the medusa TriangleMesh to libigl's V (Nx3 double) and F (Mx3 int).
        void to_eigen(const geometry::TriangleMesh& mesh,
                      Eigen::MatrixXd& V,
                      Eigen::MatrixXi& F)
        {
            V.resize(static_cast<Eigen::Index>(mesh.vertices.size()), 3);
            for (Eigen::Index i = 0; i < V.rows(); ++i)
            {
                const auto& v = mesh.vertices[static_cast<std::size_t>(i)];
                V(i, 0) = static_cast<double>(v.x);
                V(i, 1) = static_cast<double>(v.y);
                V(i, 2) = static_cast<double>(v.z);
            }

            F.resize(static_cast<Eigen::Index>(mesh.faces.size()), 3);
            for (Eigen::Index i = 0; i < F.rows(); ++i)
            {
                const auto& f = mesh.faces[static_cast<std::size_t>(i)];
                F(i, 0) = static_cast<int>(f.x);
                F(i, 1) = static_cast<int>(f.y);
                F(i, 2) = static_cast<int>(f.z);
            }
        }
    } // namespace

    ScalarField solveHarmonicField(const geometry::TriangleMesh& mesh,
                                   const ScalarFieldConfig& config)
    {
        ScalarField result;

        if (!mesh.isValid())
        {
            MEDUSA_WARN("ScalarField: mesh is empty");
            return result;
        }

        const int axis = std::clamp(config.up_axis, 0, 2);
        const float h_min = mesh.bounds.min[axis];
        const float h_max = mesh.bounds.max[axis];
        const float h_range = h_max - h_min;

        if (h_range < 1e-6f)
        {
            MEDUSA_WARN("ScalarField: mesh has zero extent along up-axis {}", axis);
            return result;
        }

        const float bottom_thresh = h_min + config.bottom_band_fraction * h_range;
        const float top_thresh    = h_max - config.top_band_fraction    * h_range;

        // ---- 1. Collect Dirichlet boundary vertex indices ----
        std::vector<int> bottom_idx;
        std::vector<int> top_idx;
        bottom_idx.reserve(mesh.vertices.size() / 16);
        top_idx.reserve(mesh.vertices.size() / 16);

        for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
        {
            const float h = mesh.vertices[i][axis];
            if (h <= bottom_thresh) bottom_idx.push_back(static_cast<int>(i));
            else if (h >= top_thresh) top_idx.push_back(static_cast<int>(i));
        }

        if (bottom_idx.empty() || top_idx.empty())
        {
            MEDUSA_WARN("ScalarField: empty boundary set (bottom={}, top={}) — "
                        "increase band fractions",
                        bottom_idx.size(), top_idx.size());
            return result;
        }

        // ---- 2. Build Eigen / libigl matrices ----
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        to_eigen(mesh, V, F);

        Eigen::SparseMatrix<double> L;
        igl::cotmatrix(V, F, L);  // negative semi-definite cotangent Laplacian

        // We solve  L * phi = 0  with phi(bottom) = 0, phi(top) = 1.
        // libigl's min_quad_with_fixed minimises 0.5 * x^T Q x + x^T B,
        // so we pass Q = -L (positive semi-definite) and B = 0.
        // The unconstrained minimiser of x^T (-L) x is the harmonic extension.
        Eigen::SparseMatrix<double> Q = -L;

        Eigen::VectorXi known(static_cast<Eigen::Index>(bottom_idx.size() + top_idx.size()));
        Eigen::VectorXd Y(known.size());
        Eigen::Index k = 0;
        for (int idx : bottom_idx) { known(k) = idx; Y(k) = 0.0; ++k; }
        for (int idx : top_idx)    { known(k) = idx; Y(k) = 1.0; ++k; }

        Eigen::VectorXd B = Eigen::VectorXd::Zero(V.rows());
        Eigen::SparseMatrix<double> Aeq;   // no linear equality constraints
        Eigen::VectorXd Beq;

        igl::min_quad_with_fixed_data<double> data;
        if (!igl::min_quad_with_fixed_precompute(Q, known, Aeq, /*pd=*/true, data))
        {
            MEDUSA_ERROR("ScalarField: min_quad_with_fixed_precompute failed");
            return result;
        }

        Eigen::VectorXd phi;
        if (!igl::min_quad_with_fixed_solve(data, B, Y, Beq, phi))
        {
            MEDUSA_ERROR("ScalarField: min_quad_with_fixed_solve failed");
            return result;
        }

        // ---- 3. Pack result ----
        result.phi.resize(static_cast<std::size_t>(phi.size()));
        result.phi_min =  std::numeric_limits<float>::infinity();
        result.phi_max = -std::numeric_limits<float>::infinity();
        for (Eigen::Index i = 0; i < phi.size(); ++i)
        {
            const auto v = static_cast<float>(phi(i));
            result.phi[static_cast<std::size_t>(i)] = v;
            result.phi_min = std::min(result.phi_min, v);
            result.phi_max = std::max(result.phi_max, v);
        }
        result.valid = true;

        // ---- 4. Per-face gradient magnitude |grad Phi| ----
        // libigl returns G as a (3F x V) sparse operator. G * phi gives a
        // 3F-vector laid out as [gx_0..gx_{F-1}, gy_0..gy_{F-1}, gz_0..gz_{F-1}].
        // We only need the per-face norm for adaptive iso-step sizing.
        Eigen::SparseMatrix<double> G;
        igl::grad(V, F, G);
        const Eigen::VectorXd grad_flat = G * phi;
        const Eigen::Index Fn = F.rows();

        result.grad_norm_face.resize(static_cast<std::size_t>(Fn));
        result.grad_face.resize(static_cast<std::size_t>(Fn));
        double gmin = std::numeric_limits<double>::infinity();
        double gmax = 0.0;
        double gsum = 0.0;
        for (Eigen::Index fi = 0; fi < Fn; ++fi)
        {
            const double gx = grad_flat(fi);
            const double gy = grad_flat(fi + Fn);
            const double gz = grad_flat(fi + 2 * Fn);
            const double g = std::sqrt(gx * gx + gy * gy + gz * gz);
            result.grad_norm_face[static_cast<std::size_t>(fi)] = static_cast<float>(g);
            result.grad_face[static_cast<std::size_t>(fi)] = glm::vec3(
                static_cast<float>(gx),
                static_cast<float>(gy),
                static_cast<float>(gz));
            gmin = std::min(gmin, g);
            gmax = std::max(gmax, g);
            gsum += g;
        }
        result.grad_min  = static_cast<float>(gmin);
        result.grad_max  = static_cast<float>(gmax);
        result.grad_mean = static_cast<float>(gsum / std::max<Eigen::Index>(Fn, 1));

        MEDUSA_INFO("ScalarField: harmonic Phi solved on {} vertices, {} faces "
                    "(|bottom|={}, |top|={}, range=[{:.4f}, {:.4f}], "
                    "|grad|=[{:.4f}, {:.4f}], mean={:.4f} [1/mm])",
                    V.rows(), F.rows(), bottom_idx.size(), top_idx.size(),
                    result.phi_min, result.phi_max,
                    result.grad_min, result.grad_max, result.grad_mean);

        // ---- 5. Diagnostic: Phi distribution per up-axis bucket ----
        // Splits the bounding box height into 10 equal slabs and reports the
        // Phi range that ended up in each. On a "stem + crossbar" geometry
        // the upper buckets compress into a thin Phi-band -> uniform-dphi
        // sampling starves them. This log makes that visible at INFO level.
        {
            constexpr int kBuckets = 10;
            std::array<float, kBuckets> b_min{};
            std::array<float, kBuckets> b_max{};
            std::array<int,   kBuckets> b_cnt{};
            for (int i = 0; i < kBuckets; ++i)
            {
                b_min[i] =  std::numeric_limits<float>::infinity();
                b_max[i] = -std::numeric_limits<float>::infinity();
                b_cnt[i] = 0;
            }
            for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
            {
                const float h = mesh.vertices[i][axis];
                int b = static_cast<int>((h - h_min) / h_range * kBuckets);
                b = std::clamp(b, 0, kBuckets - 1);
                b_min[b] = std::min(b_min[b], result.phi[i]);
                b_max[b] = std::max(b_max[b], result.phi[i]);
                ++b_cnt[b];
            }
            MEDUSA_INFO("ScalarField: Phi distribution along up-axis (bucket: y_range -> phi_range, vertex_count):");
            for (int b = 0; b < kBuckets; ++b)
            {
                const float y0 = h_min + (static_cast<float>(b)     / kBuckets) * h_range;
                const float y1 = h_min + (static_cast<float>(b + 1) / kBuckets) * h_range;
                if (b_cnt[b] == 0)
                {
                    MEDUSA_INFO("  bucket[{}] y=[{:.2f},{:.2f}]  empty", b, y0, y1);
                }
                else
                {
                    MEDUSA_INFO("  bucket[{}] y=[{:.2f},{:.2f}]  phi=[{:.3f},{:.3f}]  span={:.3f}  verts={}",
                                b, y0, y1, b_min[b], b_max[b], b_max[b] - b_min[b], b_cnt[b]);
                }
            }
        }

        return result;
    }
} // namespace slicing::crystal
