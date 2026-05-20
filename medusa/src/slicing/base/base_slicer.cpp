/**
 * @file base_slicer.cpp
 * @brief Implementation of the Base slicing algorithm with overhang detection
 *        and queue-based multi-directional slicing.
 */

#include "base_slicer.h"
#include "triangle_mesh.h"
#include "logger.h"

#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace slicing
{
    BaseSlicer::BaseSlicer(const BaseParams& params)
        : mParams(params)
    {
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Coordinate helpers
    // ═════════════════════════════════════════════════════════════════════════

    void BaseSlicer::computeUV(const glm::vec3& growth, glm::vec3& u, glm::vec3& v)
    {
        // Pick a reference vector that is not parallel to growth.
        glm::vec3 ref = (std::abs(glm::dot(growth, glm::vec3(0.0f, 1.0f, 0.0f))) > 0.9f)
                        ? glm::vec3(1.0f, 0.0f, 0.0f)
                        : glm::vec3(0.0f, 1.0f, 0.0f);
        u = glm::normalize(glm::cross(growth, ref));
        v = glm::normalize(glm::cross(u, growth));
    }

    BaseSlicer::PlaneBounds BaseSlicer::computeBounds(
        const std::vector<glm::vec3>& points,
        const glm::vec3& u, const glm::vec3& v)
    {
        PlaneBounds b;
        if (points.empty()) return b;

        b.uMin =  std::numeric_limits<float>::max();
        b.uMax =  std::numeric_limits<float>::lowest();
        b.vMin =  std::numeric_limits<float>::max();
        b.vMax =  std::numeric_limits<float>::lowest();

        for (const auto& p : points)
        {
            float pu = glm::dot(p, u);
            float pv = glm::dot(p, v);
            b.uMin = std::min(b.uMin, pu);
            b.uMax = std::max(b.uMax, pu);
            b.vMin = std::min(b.vMin, pv);
            b.vMax = std::max(b.vMax, pv);
        }
        b.valid = true;
        return b;
    }

    /**
     * @brief Clips a 3D segment to the allowed 2D bounding box using Liang-Barsky algorithm.
     * @param p0 First endpoint (modified in-place if clipped)
     * @param p1 Second endpoint (modified in-place if clipped)
     * @param u U-axis basis vector
     * @param v V-axis basis vector
     * @param allowed Bounding box in UV space
     * @return true if segment (or portion of it) lies inside bounds, false if completely outside
     */
    bool BaseSlicer::clipSegment(glm::vec3& p0, glm::vec3& p1,
                                  const glm::vec3& u, const glm::vec3& v,
                                  const PlaneBounds& allowed)
    {
        // Project endpoints to UV space
        float u0 = glm::dot(p0, u);
        float v0 = glm::dot(p0, v);
        float u1 = glm::dot(p1, u);
        float v1 = glm::dot(p1, v);

        // Liang-Barsky parameters: t ∈ [t_min, t_max] is the visible portion
        float t_min = 0.0f;
        float t_max = 1.0f;
        float du = u1 - u0;
        float dv = v1 - v0;

        // Helper: clip against a single boundary edge
        auto clipEdge = [&](float p, float q) -> bool
        {
            constexpr float eps = 1e-9f;
            if (std::abs(p) < eps)
            {
                // Line is parallel to this boundary
                return q >= 0.0f;  // Inside if q >= 0, otherwise outside
            }
            float t = q / p;
            if (p < 0.0f)
            {
                // Entering the half-space
                if (t > t_max) return false;  // Segment exits before entering
                if (t > t_min) t_min = t;
            }
            else
            {
                // Leaving the half-space
                if (t < t_min) return false;  // Segment enters after leaving
                if (t < t_max) t_max = t;
            }
            return true;
        };

        // Clip against all four boundaries
        if (!clipEdge(-du, u0 - allowed.uMin)) return false;  // u >= uMin
        if (!clipEdge( du, allowed.uMax - u0)) return false;  // u <= uMax
        if (!clipEdge(-dv, v0 - allowed.vMin)) return false;  // v >= vMin
        if (!clipEdge( dv, allowed.vMax - v0)) return false;  // v <= vMax

        // Apply clipping to 3D segment
        glm::vec3 dir = p1 - p0;
        p1 = p0 + dir * t_max;
        p0 = p0 + dir * t_min;

        return true;
    }

    void BaseSlicer::filterPoints(std::vector<glm::vec3>& points,
                                   std::vector<glm::vec3>& normals,
                                   const glm::vec3& u, const glm::vec3& v,
                                   const PlaneBounds& allowed)
    {
        if (points.size() != normals.size() || points.size() % 2 != 0) return;

        std::vector<glm::vec3> clippedPts;
        std::vector<glm::vec3> clippedNrm;
        clippedPts.reserve(points.size());
        clippedNrm.reserve(normals.size());

        for (size_t i = 0; i + 1 < points.size(); i += 2)
        {
            glm::vec3 p0 = points[i];
            glm::vec3 p1 = points[i + 1];

            if (clipSegment(p0, p1, u, v, allowed))
            {
                // Segment (or portion of it) lies inside allowed region
                clippedPts.push_back(p0);
                clippedPts.push_back(p1);
                // Keep original normals (clipping only affects endpoints, not orientation)
                clippedNrm.push_back(normals[i]);
                clippedNrm.push_back(normals[i + 1]);
            }
        }

        points = std::move(clippedPts);
        normals = std::move(clippedNrm);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Plane–mesh intersection
    // ═════════════════════════════════════════════════════════════════════════

    void BaseSlicer::intersectMeshWithPlane(
        const geometry::TriangleMesh& mesh,
        const glm::vec3& planePoint,
        const glm::vec3& planeNormal,
        std::vector<glm::vec3>& points,
        std::vector<glm::vec3>& normals)
    {
        points.clear();
        normals.clear();

        for (size_t fi = 0; fi < mesh.faces.size(); ++fi)
        {
            const auto& face = mesh.faces[fi];
            const glm::vec3& v0 = mesh.vertices[face.x];
            const glm::vec3& v1 = mesh.vertices[face.y];
            const glm::vec3& v2 = mesh.vertices[face.z];

            float d0 = glm::dot(v0 - planePoint, planeNormal);
            float d1 = glm::dot(v1 - planePoint, planeNormal);
            float d2 = glm::dot(v2 - planePoint, planeNormal);

            glm::vec3 intersections[2];
            glm::vec3 interpNormals[2];
            int count = 0;

            auto tryEdge = [&](const glm::vec3& a, const glm::vec3& b,
                               uint32_t ia, uint32_t ib,
                               float da, float db)
            {
                if (count >= 2) return;
                if ((da > 0.0f) != (db > 0.0f))
                {
                    float t = da / (da - db);
                    intersections[count] = a + t * (b - a);
                    glm::vec3 na = mesh.normals[ia];
                    glm::vec3 nb = mesh.normals[ib];
                    glm::vec3 n = na + t * (nb - na);
                    float len = glm::length(n);
                    interpNormals[count] = (len > 1e-8f) ? n / len : planeNormal;
                    ++count;
                }
            };

            tryEdge(v0, v1, face.x, face.y, d0, d1);
            tryEdge(v1, v2, face.y, face.z, d1, d2);
            tryEdge(v2, v0, face.z, face.x, d2, d0);

            if (count == 2)
            {
                points.push_back(intersections[0]);
                points.push_back(intersections[1]);
                normals.push_back(interpNormals[0]);
                normals.push_back(interpNormals[1]);
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Single slicing run
    // ═════════════════════════════════════════════════════════════════════════

    void BaseSlicer::executeRun(const geometry::TriangleMesh& mesh,
                                 const SliceRun& run,
                                 std::vector<Layer>& layers,
                                 std::queue<SliceRun>& pending,
                                 uint32_t& layerIdx,
                                 uint32_t& nextBranchId)
    {
        const float thickness = mParams.layer_thickness;
        const float tolerance = thickness *
            std::tan(glm::radians(std::clamp(mParams.overhang_angle,
                                             BaseParams::OVERHANG_ANGLE_MIN,
                                             BaseParams::OVERHANG_ANGLE_MAX)));

        glm::vec3 u, v;
        computeUV(run.growth_dir, u, v);

        // Determine up_axis from growth direction (dominant axis component).
        int upAxis = 0;
        float maxComp = 0.0f;
        for (int a = 0; a < 3; ++a)
        {
            float c = std::abs(run.growth_dir[a]);
            if (c > maxComp) { maxComp = c; upAxis = a; }
        }

        // Find mesh extent along growth direction.
        float minH = std::numeric_limits<float>::max();
        float maxH = std::numeric_limits<float>::lowest();
        for (const auto& vert : mesh.vertices)
        {
            float h = glm::dot(vert, run.growth_dir);
            minH = std::min(minH, h);
            maxH = std::max(maxH, h);
        }

        float startH = run.start_h + thickness * 0.5f;
        if (startH > maxH) return;

        PlaneBounds prevBounds; // .valid = false → no previous layer yet

        // Allowed bounds — tightened when overhangs are detected.
        PlaneBounds allowed;
        allowed.uMin = std::numeric_limits<float>::lowest();
        allowed.uMax = std::numeric_limits<float>::max();
        allowed.vMin = std::numeric_limits<float>::lowest();
        allowed.vMax = std::numeric_limits<float>::max();
        allowed.valid = true;

        // Log initial allowed bounds for branches
        if (run.branch_id > 0)
        {
            MEDUSA_DEBUG("BaseSlicer Branch INIT: id={}, allowed=({:.2f},{:.2f},{:.2f},{:.2f}) "
                        "(WARNING: infinite bounds - no initial clipping!)",
                        run.branch_id,
                        allowed.uMin, allowed.uMax, allowed.vMin, allowed.vMax);
        }

        // Track which sides have already been queued in this run.
        // Once an overhang is detected in a direction, all geometry
        // beyond that boundary belongs to the child run.
        bool queuedUPlus  = false;
        bool queuedUMinus = false;
        bool queuedVPlus  = false;
        bool queuedVMinus = false;

        bool firstLayer = true;

        for (float h = startH; h <= maxH; h += thickness)
        {
            glm::vec3 planePoint = run.growth_dir * h;

            std::vector<glm::vec3> points, normals;
            intersectMeshWithPlane(mesh, planePoint, run.growth_dir, points, normals);

            if (points.empty()) continue;

            PlaneBounds currentBounds = computeBounds(points, u, v);
            if (!currentBounds.valid) continue;

            // Log first layer for branches to show mesh bounds vs allowed
            if (firstLayer && run.branch_id > 0)
            {
                MEDUSA_DEBUG("BaseSlicer Branch FIRST LAYER: id={}, h={:.3f}, "
                            "mesh_bounds_in_uv=({:.2f},{:.2f},{:.2f},{:.2f}), "
                            "allowed=({:.2f},{:.2f},{:.2f},{:.2f})",
                            run.branch_id, h,
                            currentBounds.uMin, currentBounds.uMax,
                            currentBounds.vMin, currentBounds.vMax,
                            allowed.uMin, allowed.uMax, allowed.vMin, allowed.vMax);
                firstLayer = false;
            }

            // ─── Overhang detection: tighten allowed bounds & queue ───
            if (prevBounds.valid)
            {
                if (!queuedUPlus &&
                    currentBounds.uMax > prevBounds.uMax + tolerance)
                {
                    allowed.uMax = prevBounds.uMax;
                    pending.push({u, prevBounds.uMax, nextBranchId++});
                    queuedUPlus = true;
                    MEDUSA_DEBUG("BaseSlicer Branch SPAWN: id={}, dir=({:.2f},{:.2f},{:.2f}), "
                                "start_h={:.3f}, overhang_type=+U, parent_bounds=({:.2f},{:.2f},{:.2f},{:.2f})",
                                nextBranchId - 1, u.x, u.y, u.z, prevBounds.uMax,
                                prevBounds.uMin, prevBounds.uMax, prevBounds.vMin, prevBounds.vMax);
                }
                if (!queuedUMinus &&
                    currentBounds.uMin < prevBounds.uMin - tolerance)
                {
                    allowed.uMin = prevBounds.uMin;
                    pending.push({-u, -prevBounds.uMin, nextBranchId++});
                    queuedUMinus = true;
                    MEDUSA_DEBUG("BaseSlicer Branch SPAWN: id={}, dir=({:.2f},{:.2f},{:.2f}), "
                                "start_h={:.3f}, overhang_type=-U, parent_bounds=({:.2f},{:.2f},{:.2f},{:.2f})",
                                nextBranchId - 1, -u.x, -u.y, -u.z, -prevBounds.uMin,
                                prevBounds.uMin, prevBounds.uMax, prevBounds.vMin, prevBounds.vMax);
                }
                if (!queuedVPlus &&
                    currentBounds.vMax > prevBounds.vMax + tolerance)
                {
                    allowed.vMax = prevBounds.vMax;
                    pending.push({v, prevBounds.vMax, nextBranchId++});
                    queuedVPlus = true;
                    MEDUSA_DEBUG("BaseSlicer Branch SPAWN: id={}, dir=({:.2f},{:.2f},{:.2f}), "
                                "start_h={:.3f}, overhang_type=+V, parent_bounds=({:.2f},{:.2f},{:.2f},{:.2f})",
                                nextBranchId - 1, v.x, v.y, v.z, prevBounds.vMax,
                                prevBounds.uMin, prevBounds.uMax, prevBounds.vMin, prevBounds.vMax);
                }
                if (!queuedVMinus &&
                    currentBounds.vMin < prevBounds.vMin - tolerance)
                {
                    allowed.vMin = prevBounds.vMin;
                    pending.push({-v, -prevBounds.vMin, nextBranchId++});
                    queuedVMinus = true;
                    MEDUSA_DEBUG("BaseSlicer Branch SPAWN: id={}, dir=({:.2f},{:.2f},{:.2f}), "
                                "start_h={:.3f}, overhang_type=-V, parent_bounds=({:.2f},{:.2f},{:.2f},{:.2f})",
                                nextBranchId - 1, -v.x, -v.y, -v.z, -prevBounds.vMin,
                                prevBounds.uMin, prevBounds.uMax, prevBounds.vMin, prevBounds.vMax);
                }
            }

            // ─── Filter segments to the allowed region ───────────────
            // Segments whose midpoint falls outside the allowed bounds
            // belong to child runs and are removed from this layer.

            // DIAGNOSTIC LOG: pre-filter state
            size_t segmentsBeforeFilter = points.size() / 2;
            MEDUSA_DEBUG("BaseSlicer Layer (branch={}): pre-filter bounds=({:.2f},{:.2f},{:.2f},{:.2f}), "
                        "allowed=({:.2f},{:.2f},{:.2f},{:.2f}), segments={}",
                        run.branch_id,
                        currentBounds.uMin, currentBounds.uMax, currentBounds.vMin, currentBounds.vMax,
                        allowed.uMin, allowed.uMax, allowed.vMin, allowed.vMax,
                        segmentsBeforeFilter);

            filterPoints(points, normals, u, v, allowed);

            if (points.empty()) continue;

            // Update prevBounds from the filtered (clipped) contour so
            // subsequent overhang checks compare against this run's
            // actual footprint, not the full mesh cross-section.
            prevBounds = computeBounds(points, u, v);
            if (!prevBounds.valid) continue;

            // Clamp prevBounds to allowed bounds to prevent drift.
            // This ensures that even if a segment slips through the filter,
            // prevBounds cannot grow beyond the allowed region.
            prevBounds.uMin = std::max(prevBounds.uMin, allowed.uMin);
            prevBounds.uMax = std::min(prevBounds.uMax, allowed.uMax);
            prevBounds.vMin = std::max(prevBounds.vMin, allowed.vMin);
            prevBounds.vMax = std::min(prevBounds.vMax, allowed.vMax);

            // DIAGNOSTIC LOG: post-filter state
            size_t segmentsAfterFilter = points.size() / 2;
            MEDUSA_DEBUG("BaseSlicer Layer (branch={}): post-filter segments={}, "
                        "post-filter bounds=({:.2f},{:.2f},{:.2f},{:.2f})",
                        run.branch_id,
                        segmentsAfterFilter,
                        prevBounds.uMin, prevBounds.uMax, prevBounds.vMin, prevBounds.vMax);

            // ─── Add layer ───────────────────────────────────────────
            Layer layer;
            layer.index = layerIdx++;
            layer.branch_id = run.branch_id;
            layer.thickness = thickness;
            layer.up_axis = upAxis;
            layer.contour_points = std::move(points);
            layer.contour_normals = std::move(normals);
            layers.push_back(std::move(layer));
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Main slicing entry point
    // ═════════════════════════════════════════════════════════════════════════

    std::vector<Layer> BaseSlicer::slice(const geometry::TriangleMesh& mesh)
    {
        std::vector<Layer> layers;
        mCoverage = 0.0f;

        if (!mesh.isValid())
        {
            MEDUSA_WARN("BaseSlicer: empty or invalid mesh");
            return layers;
        }

        const int upAxis = mParams.up_axis;
        const float hMin = mesh.bounds.min[upAxis];
        const float hMax = mesh.bounds.max[upAxis];
        const float height = hMax - hMin;

        if (height < 1e-6f)
        {
            MEDUSA_WARN("BaseSlicer: mesh has zero height along up_axis {}", upAxis);
            return layers;
        }

        uint32_t layerIdx = 0;
        uint32_t nextBranchId = 1; // 0 = initial (stem) run
        std::queue<SliceRun> pending;

        // Initial run: build plate upward along up_axis.
        glm::vec3 initDir(0.0f);
        initDir[upAxis] = 1.0f;
        pending.push({initDir, hMin, 0});

        const int maxIter = std::clamp(mParams.max_iterations,
                                       BaseParams::MAX_ITERATIONS_MIN,
                                       BaseParams::MAX_ITERATIONS_MAX);
        int iteration = 0;

        while (!pending.empty() && iteration < maxIter)
        {
            SliceRun run = pending.front();
            pending.pop();

            MEDUSA_INFO("BaseSlicer: run {} (branch {}) — dir=({:.2f},{:.2f},{:.2f}), start_h={:.3f}",
                        iteration, run.branch_id,
                        run.growth_dir.x, run.growth_dir.y, run.growth_dir.z,
                        run.start_h);

            executeRun(mesh, run, layers, pending, layerIdx, nextBranchId);
            ++iteration;
        }

        if (!pending.empty())
        {
            MEDUSA_WARN("BaseSlicer: reached max iterations ({}), {} runs remaining",
                        maxIter, pending.size());
        }

        // Coverage estimate: ratio of generated layers to the expected
        // planar layer count. Multi-directional runs may exceed 1.0.
        const auto expectedLayers = static_cast<uint32_t>(
            std::ceil(height / mParams.layer_thickness));
        mCoverage = (expectedLayers > 0)
            ? std::min(1.0f, static_cast<float>(layers.size())
                           / static_cast<float>(expectedLayers))
            : 0.0f;

        MEDUSA_INFO("BaseSlicer: {} total layers, {} branches, {} runs (coverage={:.1f}%)",
                    layers.size(), nextBranchId, iteration, mCoverage * 100.0f);
        return layers;
    }

} // namespace slicing
