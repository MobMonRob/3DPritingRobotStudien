/**
 * @file infill_generator.cpp
 * @brief Implementation of 100% non-planar infill generation using scanline fill.
 */

#include "infill_generator.h"
#include "logger.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace slicing
{
    void InfillGenerator::generate(std::vector<Layer>& layers, float lineSpacing)
    {
        size_t totalInfillSegments = 0;

        for (auto& layer : layers)
        {
            const size_t contourSize = layer.contour_points.size();
            if (contourSize < 2)
            {
                MEDUSA_DEBUG("InfillGenerator: layer {} skipped (only {} contour point(s))",
                             layer.index, contourSize);
                continue;
            }

            float spacing = lineSpacing > 0.0f ? lineSpacing : layer.thickness;
            MEDUSA_DEBUG("InfillGenerator: layer {} — spacing={:.3f}mm, {} contour points",
                         layer.index, spacing, contourSize);

            // Record how many contour points exist before we append infill.
            layer.contour_count = layer.contour_points.size();

            // Build boundary edges from contour segment pairs (before we append infill).
            // Each pair (pts[2i], pts[2i+1]) is one boundary edge from a triangle-plane intersection.
            struct Edge
            {
                glm::vec3 a, b;
            };
            std::vector<Edge> edges;
            edges.reserve(contourSize / 2);
            for (size_t i = 0; i + 1 < contourSize; i += 2)
            {
                edges.push_back({layer.contour_points[i], layer.contour_points[i + 1]});
            }

            if (edges.empty())
            {
                continue;
            }

            // Find bounding box of contour in all 3 axes.
            glm::vec3 bbMin(std::numeric_limits<float>::max());
            glm::vec3 bbMax(std::numeric_limits<float>::lowest());
            for (const auto& e : edges)
            {
                for (int a = 0; a < 3; ++a)
                {
                    bbMin[a] = std::min({bbMin[a], e.a[a], e.b[a]});
                    bbMax[a] = std::max({bbMax[a], e.a[a], e.b[a]});
                }
            }

            glm::vec3 span = bbMax - bbMin;

            // P5 fix: derive axes from the stored up_axis instead of guessing via
            // smallest bounding box span (which breaks for flat or L-shaped cross sections).
            // normalAxis = the slicing up-axis (constant for all points in this layer).
            // sweepAxis  = axis along which scanlines are spaced.
            // fillAxis   = axis along which each scanline extends.
            const int normalAxis = layer.up_axis;
            const int sweepAxis  = (normalAxis + 1) % 3;
            const int fillAxis   = (normalAxis + 2) % 3;
            (void)span; // retained for bounding box; no longer used for axis selection

            float sweepMin = bbMin[sweepAxis];
            float sweepMax = bbMax[sweepAxis];

            if (sweepMax - sweepMin < 1e-6f)
            {
                MEDUSA_WARN("InfillGenerator: layer {} — degenerate sweep span (< 1e-6), infill skipped",
                            layer.index);
                continue;
            }

            // Constant coordinate along the normal axis.
            const float normalVal = layer.contour_points[0][normalAxis];

            // Infill normal: axis-aligned along the normal axis.
            glm::vec3 infillNormal(0.0f);
            infillNormal[normalAxis] = 1.0f;

            // Helper to build a 3D point from axis values.
            auto makePoint = [&](float sweepVal, float fillVal) -> glm::vec3 {
                glm::vec3 pt;
                pt[normalAxis] = normalVal;
                pt[sweepAxis]  = sweepVal;
                pt[fillAxis]   = fillVal;
                return pt;
            };

            // Scanline fill: sweep at regular spacing, alternating direction.
            bool reverseDir = false;

            for (float s = sweepMin + spacing * 0.5f; s < sweepMax; s += spacing)
            {
                // Find fill-axis intersections of this scanline with boundary edges.
                std::vector<float> fillHits;
                for (const auto& e : edges)
                {
                    float sa = e.a[sweepAxis];
                    float sb = e.b[sweepAxis];
                    if ((sa < s) != (sb < s))
                    {
                        float t = (s - sa) / (sb - sa);
                        fillHits.push_back(e.a[fillAxis] + t * (e.b[fillAxis] - e.a[fillAxis]));
                    }
                }

                if (fillHits.size() < 2)
                {
                    reverseDir = !reverseDir;
                    continue;
                }

                std::sort(fillHits.begin(), fillHits.end());

                // Even-odd fill rule: interior spans are between pairs (0,1), (2,3), ...
                if (reverseDir)
                {
                    for (size_t i = fillHits.size(); i >= 2; i -= 2)
                    {
                        layer.contour_points.push_back(makePoint(s, fillHits[i - 1]));
                        layer.contour_points.push_back(makePoint(s, fillHits[i - 2]));
                        layer.contour_normals.push_back(infillNormal);
                        layer.contour_normals.push_back(infillNormal);
                        ++totalInfillSegments;
                    }
                }
                else
                {
                    for (size_t i = 0; i + 1 < fillHits.size(); i += 2)
                    {
                        layer.contour_points.push_back(makePoint(s, fillHits[i]));
                        layer.contour_points.push_back(makePoint(s, fillHits[i + 1]));
                        layer.contour_normals.push_back(infillNormal);
                        layer.contour_normals.push_back(infillNormal);
                        ++totalInfillSegments;
                    }
                }

                reverseDir = !reverseDir;
            }
        }

        MEDUSA_INFO("InfillGenerator: generated {} infill segments across {} layers",
                    totalInfillSegments, layers.size());
    }
} // namespace slicing
