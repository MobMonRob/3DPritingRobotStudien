/**
 * @file phi_overlay_renderer.h
 * @brief Renders the Crystal harmonic scalar field Phi as a heatmap on the mesh.
 *
 * Used by the application to visually validate Step 2 of Crystal (the
 * harmonic field solve). Operates on geometry::TriangleMesh + a per-vertex
 * Phi array; uploads an indexed VBO and renders triangles colored by a
 * viridis-style colormap.
 */

#ifndef MEDUSA_SRC_GRAPHICS_PHI_OVERLAY_RENDERER_H_
#define MEDUSA_SRC_GRAPHICS_PHI_OVERLAY_RENDERER_H_

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "shader_program.h"

namespace geometry { struct TriangleMesh; }

namespace graphics
{
    /**
     * @brief OpenGL renderer for a per-vertex scalar field on a triangle mesh.
     */
    class PhiOverlayRenderer
    {
    public:
        PhiOverlayRenderer() = default;
        ~PhiOverlayRenderer();

        PhiOverlayRenderer(const PhiOverlayRenderer&) = delete;
        PhiOverlayRenderer& operator=(const PhiOverlayRenderer&) = delete;

        /// Initialises GPU resources and compiles shaders. Returns true on success.
        [[nodiscard]] bool initialize();

        /// Uploads (mesh, phi) to the GPU. phi.size() must equal mesh.numVertices().
        /// Empty mesh or size mismatch -> overlay is cleared.
        void upload(const geometry::TriangleMesh& mesh, const std::vector<float>& phi,
                    float phi_min, float phi_max);

        /// Renders the overlay. No-op if no data has been uploaded.
        void render(const glm::mat4& mvp) const;

        /// Releases GPU resources and resets state.
        void clear();

        [[nodiscard]] bool hasData() const noexcept { return mIndexCount > 0; }

    private:
        ShaderProgram mShader;
        GLuint        mVao{0};
        GLuint        mVbo{0};        // interleaved: (pos.xyz, phi)
        GLuint        mEbo{0};
        GLsizei       mIndexCount{0};

        GLint         mLocMvp{-1};
        GLint         mLocPhiMin{-1};
        GLint         mLocPhiMax{-1};

        float         mPhiMin{0.0f};
        float         mPhiMax{1.0f};
    };
} // namespace graphics

#endif // MEDUSA_SRC_GRAPHICS_PHI_OVERLAY_RENDERER_H_
