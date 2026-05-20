/**
 * @file planar_slicer.cpp
 * @brief Implementation of the horizontal planar slicer.
 */

#include "planar_slicer.h"
#include "triangle_mesh.h"
#include "logger.h"

#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace slicing
{
    PlanarSlicer::PlanarSlicer(const SlicerParams& params)
        : mParams(params)
    {
    }

    std::vector<Layer> PlanarSlicer::slice(const geometry::TriangleMesh& mesh)
    {
        std::vector<Layer> layers;
        mCoverage = 0.0f;

        if (!mesh.isValid())
        {
            MEDUSA_WARN("PlanarSlicer: empty or invalid mesh");
            return layers;
        }

        const int   upAxis  = mParams.up_axis;
        const float yMin    = mesh.bounds.min[upAxis];
        const float yMax    = mesh.bounds.max[upAxis];
        const float height  = yMax - yMin;

        if (height < 1e-6f)
        {
            MEDUSA_WARN("PlanarSlicer: mesh has zero height along up_axis {}", upAxis);
            return layers;
        }

        const float thickness = mParams.layer_thickness;
        // Start half a layer above the bottom, end half a layer below the top
        const float startY = yMin + thickness * 0.5f;
        const uint32_t numLayers = static_cast<uint32_t>(std::ceil(height / thickness));

        layers.reserve(numLayers);

        MEDUSA_INFO("PlanarSlicer: slicing {} layers (y={:.3f} to {:.3f}, thickness={:.3f}mm)",
                    numLayers, yMin, yMax, thickness);

        for (uint32_t i = 0; i < numLayers; ++i)
        {
            float y = startY + static_cast<float>(i) * thickness;
            if (y > yMax)
            {
                y = yMax - 1e-6f; // Clamp to stay within mesh
            }

            Layer layer;
            layer.index = i;
            layer.thickness = thickness;

            intersectPlane(mesh, y, upAxis, layer.contour_points, layer.contour_normals);

            if (!layer.contour_points.empty())
            {
                layer.up_axis = upAxis;
                layers.push_back(std::move(layer));
            }

            mCoverage = static_cast<float>(i + 1) / static_cast<float>(numLayers);
        }

        // P6: coverage = fraction of height-based slices that had actual geometry.
        // This is more accurate than always returning 1.0 — sparse meshes or meshes
        // with internal voids will report < 1.0 correctly.
        mCoverage = (numLayers > 0)
            ? static_cast<float>(layers.size()) / static_cast<float>(numLayers)
            : 1.0f;
        MEDUSA_INFO("PlanarSlicer: generated {} non-empty layers (coverage={:.1f}%)",
                    layers.size(), mCoverage * 100.0f);
        return layers;
    }

    void PlanarSlicer::intersectPlane(const geometry::TriangleMesh& mesh, float planePos, int upAxis,
                                       std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals)
    {
        points.clear();
        normals.clear();

        // --- Step 1: Collect raw segment pairs from triangle-plane intersections ---
        struct RawEdge { glm::vec3 a, b; };
        std::vector<RawEdge> raw;
        raw.reserve(mesh.faces.size() / 4);

        for (const auto& face : mesh.faces)
        {
            const glm::vec3& v0 = mesh.vertices[face.x];
            const glm::vec3& v1 = mesh.vertices[face.y];
            const glm::vec3& v2 = mesh.vertices[face.z];

            const float d0 = v0[upAxis] - planePos;
            const float d1 = v1[upAxis] - planePos;
            const float d2 = v2[upAxis] - planePos;

            glm::vec3 isect[2];
            int n = 0;

            // P3 fix: when the b-endpoint lies exactly on the plane, skip this crossing.
            // The adjacent edge will catch that vertex as its a-endpoint, preventing
            // double-counting at corners where a vertex is shared by two edges.
            auto tryEdge = [&](const glm::vec3& a, const glm::vec3& b, float da, float db)
            {
                if (n >= 2) return;
                if ((da > 0.0f) != (db > 0.0f))
                {
                    if (std::abs(db) >= 1e-6f)   // b on plane → next edge handles it
                    {
                        const float t = da / (da - db);
                        isect[n++] = a + t * (b - a);
                    }
                }
                else if (std::abs(da) < 1e-6f)   // a exactly on plane
                {
                    isect[n++] = a;
                }
            };

            tryEdge(v0, v1, d0, d1);
            tryEdge(v1, v2, d1, d2);
            tryEdge(v2, v0, d2, d0);

            if (n == 2)
                raw.push_back({isect[0], isect[1]});
        }

        if (raw.empty()) return;

        // --- Step 2: Chain segments into ordered contour rings ---
        // Greedy nearest-neighbour: for each ring, repeatedly find the unused segment
        // whose nearest endpoint continues from the current cursor position.
        // O(n²) per layer — fast enough for any realistic layer edge count.
        //
        // Snap tolerance: intersection points on shared mesh edges are computed
        // identically (same formula, same operands) so are bit-for-bit equal.
        // A small epsilon handles floating-point noise from degenerate geometry.
        static constexpr float kSnapTol = 1e-4f;           // 0.1 µm
        static constexpr float kSnapSq  = kSnapTol * kSnapTol;

        auto distSq = [](const glm::vec3& p, const glm::vec3& q) -> float
        {
            const glm::vec3 d = p - q;
            return d.x * d.x + d.y * d.y + d.z * d.z;
        };

        std::vector<bool> used(raw.size(), false);

        // --- Step 2: Chain raw segments into closed contour rings ---
        // Each disconnected contour on this layer (outer wall, holes, islands)
        // becomes its own ring. Rings are collected first; ordering happens in Step 3.
        std::vector<std::vector<glm::vec3>> rings;

        for (size_t seedIdx = 0; seedIdx < raw.size(); ++seedIdx)
        {
            if (used[seedIdx]) continue;

            std::vector<glm::vec3> ring;
            ring.reserve(32);

            used[seedIdx] = true;
            ring.push_back(raw[seedIdx].a);
            ring.push_back(raw[seedIdx].b);

            glm::vec3       cursor = raw[seedIdx].b;
            const glm::vec3 origin = raw[seedIdx].a;

            while (true)
            {
                size_t bestJ    = SIZE_MAX;
                float  bestD    = std::numeric_limits<float>::max();
                bool   bestFlip = false;

                for (size_t j = 0; j < raw.size(); ++j)
                {
                    if (used[j]) continue;
                    const float dA = distSq(cursor, raw[j].a);
                    const float dB = distSq(cursor, raw[j].b);
                    if (dA < bestD) { bestD = dA; bestJ = j; bestFlip = false; }
                    if (dB < bestD) { bestD = dB; bestJ = j; bestFlip = true;  }
                }

                if (bestJ == SIZE_MAX || bestD > kSnapSq) break;

                used[bestJ] = true;
                cursor = bestFlip ? raw[bestJ].a : raw[bestJ].b;
                ring.push_back(cursor);

                if (distSq(cursor, origin) <= kSnapSq) break;
            }

            if (ring.size() >= 2)
                rings.push_back(std::move(ring));
        }

        if (rings.empty()) return;

        if (rings.size() > 1)
        {
            MEDUSA_DEBUG("PlanarSlicer: plane pos={:.3f} — {} separate contour ring(s) found",
                         planePos, rings.size());
        }

        // --- Step 3: Reorder rings to minimise inter-ring travel-move length ---
        // Greedy nearest-neighbour over ring endpoints. Also flips a ring if
        // approaching from its back is shorter than from its front.
        // This avoids long diagonal jumps through the interior of the part when
        // a layer has multiple disconnected contours (islands, holes, etc.).
        {
            std::vector<bool>                     ringUsed(rings.size(), false);
            std::vector<std::vector<glm::vec3>>   ordered;
            ordered.reserve(rings.size());

            // Seed: pick the ring with the lowest overall bounding-box centroid
            // (approximate "bottom-left" start) so the robot begins near the origin.
            size_t seedRing = 0;
            {
                float bestCy = std::numeric_limits<float>::max();
                for (size_t r = 0; r < rings.size(); ++r)
                {
                    float cy = 0.0f;
                    for (const auto& p : rings[r]) cy += p.z; // Z = horizontal in Y-up
                    cy /= static_cast<float>(rings[r].size());
                    if (cy < bestCy) { bestCy = cy; seedRing = r; }
                }
            }

            ringUsed[seedRing] = true;
            ordered.push_back(std::move(rings[seedRing]));
            glm::vec3 cursor = ordered.back().back();

            while (ordered.size() < rings.size())
            {
                size_t bestR    = SIZE_MAX;
                float  bestD    = std::numeric_limits<float>::max();
                bool   bestFlip = false;

                for (size_t r = 0; r < rings.size(); ++r)
                {
                    if (ringUsed[r]) continue;
                    const float dFront = distSq(cursor, rings[r].front());
                    const float dBack  = distSq(cursor, rings[r].back());
                    if (dFront < bestD) { bestD = dFront; bestR = r; bestFlip = false; }
                    if (dBack  < bestD) { bestD = dBack;  bestR = r; bestFlip = true;  }
                }

                if (bestR == SIZE_MAX) break;

                ringUsed[bestR] = true;
                if (bestFlip)
                    std::reverse(rings[bestR].begin(), rings[bestR].end());
                ordered.push_back(std::move(rings[bestR]));
                cursor = ordered.back().back();
            }

            rings = std::move(ordered);
        }

        // --- Step 4: Emit all rings as consecutive segment pairs ---
        // Format: (p0,p1), (p1,p2), ..., (pN-1,pN) per ring.
        // The path planner detects the gap between ring[k] end and ring[k+1] start
        // and inserts a travel move there automatically.
        static constexpr glm::vec3 kPlaneNormal{0.0f, 1.0f, 0.0f};
        for (const auto& ring : rings)
        {
            for (size_t i = 0; i + 1 < ring.size(); ++i)
            {
                points.push_back(ring[i]);
                points.push_back(ring[i + 1]);
                normals.push_back(kPlaneNormal);
                normals.push_back(kPlaneNormal);
            }
        }
    }
} // namespace slicing
