/**
 * @file mesh_refiner.cpp
 * @brief Implementation of the Crystal mid-edge subdivision pass.
 */

#include "mesh_refiner.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

// Eigen 5.x relocated `all` into Eigen::placeholders::; libigl 2.5 still
// references `Eigen::all` directly. Shadow the symbol locally so libigl
// headers compile against the newer Eigen. Same trick used in scalar_field.cpp.
namespace Eigen { using placeholders::all; }

#include <igl/upsample.h>

#include "logger.h"

namespace slicing::crystal
{
    namespace
    {
        /// Convert medusa mesh -> libigl matrices.
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

        /// Convert libigl matrices -> medusa mesh (vertices/faces/bounds only).
        void from_eigen(const Eigen::MatrixXd& V,
                        const Eigen::MatrixXi& F,
                        geometry::TriangleMesh& mesh)
        {
            mesh.vertices.resize(static_cast<std::size_t>(V.rows()));
            mesh.bounds = {};
            for (Eigen::Index i = 0; i < V.rows(); ++i)
            {
                glm::vec3 p(static_cast<float>(V(i, 0)),
                            static_cast<float>(V(i, 1)),
                            static_cast<float>(V(i, 2)));
                mesh.vertices[static_cast<std::size_t>(i)] = p;
                mesh.bounds.expand(p);
            }
            mesh.faces.resize(static_cast<std::size_t>(F.rows()));
            for (Eigen::Index i = 0; i < F.rows(); ++i)
            {
                mesh.faces[static_cast<std::size_t>(i)] = glm::uvec3(
                    static_cast<std::uint32_t>(F(i, 0)),
                    static_cast<std::uint32_t>(F(i, 1)),
                    static_cast<std::uint32_t>(F(i, 2)));
            }
            // Adjacency tables are stale post-subdivision and Crystal's
            // current pipeline does not read them — drop instead of recompute.
            mesh.normals.clear();
            mesh.face_normals.clear();
            mesh.face_to_face.clear();
            mesh.vertex_to_faces.clear();
        }

        /// Maximum edge length over all faces. O(F).
        float maxEdgeLength(const geometry::TriangleMesh& mesh)
        {
            float m = 0.0f;
            for (const auto& f : mesh.faces)
            {
                const auto& a = mesh.vertices[f.x];
                const auto& b = mesh.vertices[f.y];
                const auto& c = mesh.vertices[f.z];
                m = std::max({m,
                              glm::length(b - a),
                              glm::length(c - b),
                              glm::length(a - c)});
            }
            return m;
        }
    } // namespace

    bool refineMesh(geometry::TriangleMesh& mesh, const MeshRefinerConfig& cfg)
    {
        if (!mesh.isValid())
        {
            MEDUSA_WARN("MeshRefiner: invalid mesh");
            return false;
        }
        if (cfg.min_edge_length <= 0.0f)
        {
            MEDUSA_WARN("MeshRefiner: min_edge_length must be > 0 (got {:.4f}), skipping",
                        cfg.min_edge_length);
            return true;
        }

        const std::size_t v0 = mesh.numVertices();
        const std::size_t f0 = mesh.numFaces();
        const float edge0 = maxEdgeLength(mesh);

        const bool low_density = static_cast<int>(v0) < cfg.min_vertex_warn_threshold;
        if (low_density)
        {
            MEDUSA_WARN("MeshRefiner: input mesh has only {} vertices (< {}). "
                        "Forcing at least one subdivision pass to give the "
                        "harmonic solver enough degrees of freedom.",
                        v0, cfg.min_vertex_warn_threshold);
        }

        if (!low_density && edge0 <= cfg.min_edge_length)
        {
            MEDUSA_INFO("MeshRefiner: max edge {:.3f} mm already <= target {:.3f} mm "
                        "({} verts, {} faces) — no subdivision needed",
                        edge0, cfg.min_edge_length, v0, f0);
            return true;
        }

        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        to_eigen(mesh, V, F);

        int iter = 0;
        float cur_edge = edge0;
        while (iter < cfg.max_iterations
               && (cur_edge > cfg.min_edge_length
                   || (low_density && iter == 0)))
        {
            Eigen::MatrixXd Vu;
            Eigen::MatrixXi Fu;
            igl::upsample(V, F, Vu, Fu);  // single 4-1 split per face
            V = std::move(Vu);
            F = std::move(Fu);

            // Recompute max edge length on the upsampled mesh. Since 4-1
            // split halves every original edge and the only new edges are
            // half-edges of the originals, the new max edge is exactly
            // cur_edge / 2 — but we measure to be safe (numerical drift,
            // future-proofing if we swap the algorithm).
            float new_edge = 0.0f;
            for (Eigen::Index i = 0; i < F.rows(); ++i)
            {
                const Eigen::Vector3d a = V.row(F(i, 0));
                const Eigen::Vector3d b = V.row(F(i, 1));
                const Eigen::Vector3d c = V.row(F(i, 2));
                new_edge = std::max({new_edge,
                                     static_cast<float>((b - a).norm()),
                                     static_cast<float>((c - b).norm()),
                                     static_cast<float>((a - c).norm())});
            }
            cur_edge = new_edge;
            ++iter;

            MEDUSA_DEBUG("MeshRefiner: iter {} -> {} verts, {} faces, max_edge={:.3f}",
                         iter, V.rows(), F.rows(), cur_edge);
        }

        from_eigen(V, F, mesh);

        const std::size_t v1 = mesh.numVertices();
        const std::size_t f1 = mesh.numFaces();
        MEDUSA_INFO("MeshRefiner: {} -> {} vertices, {} -> {} faces in {} iter(s) "
                    "(max_edge {:.3f} -> {:.3f} mm, target {:.3f} mm)",
                    v0, v1, f0, f1, iter, edge0, cur_edge, cfg.min_edge_length);

        if (cur_edge > cfg.min_edge_length)
        {
            MEDUSA_WARN("MeshRefiner: max_iterations ({}) reached but max_edge "
                        "{:.3f} mm still above target {:.3f} mm",
                        cfg.max_iterations, cur_edge, cfg.min_edge_length);
        }
        return true;
    }
} // namespace slicing::crystal
