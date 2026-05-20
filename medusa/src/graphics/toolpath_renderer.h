/**
 * @file toolpath_renderer.h
 * @brief OpenGL renderer for toolpath visualization with branch color coding.
 */

#ifndef MEDUSA_SRC_GRAPHICS_TOOLPATH_RENDERER_H_
#define MEDUSA_SRC_GRAPHICS_TOOLPATH_RENDERER_H_

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

#include "shader_program.h"

namespace slicing
{
    struct Toolpath;
}

/**
 * @brief Renders a slicing toolpath as colored lines in the 3D viewport.
 *
 * Features:
 * - Branch color coding using a colorblind-friendly palette
 * - Origin marker rendering
 * - Optional orientation vectors (TCP direction arrows)
 */
class ToolpathRenderer
{
public:
    ToolpathRenderer() = default;
    ~ToolpathRenderer();

    ToolpathRenderer(const ToolpathRenderer&) = delete;
    ToolpathRenderer& operator=(const ToolpathRenderer&) = delete;

    /**
     * @brief Initializes GPU resources and compiles shaders.
     * @return True on success.
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Uploads toolpath data to the GPU.
     * @param toolpath The toolpath to visualize.
     */
    void upload(const slicing::Toolpath& toolpath);

    /**
     * @brief Renders the toolpath.
     * @param mvp Model-View-Projection matrix.
     * @param showInfill If true, render infill lines.
     * @param showTravel If true, render travel (non-extrusion) moves.
     * @param showOrientations If true, render TCP orientation arrows.
     */
    void render(const glm::mat4& mvp, bool showInfill = true, bool showTravel = true, bool showOrientations = false) const;

    /**
     * @brief Clears GPU resources.
     */
    void clear();

    /** @brief Returns true if toolpath data has been uploaded. */
    [[nodiscard]] bool hasData() const { return mVertexCount > 0 || mInfillVertexCount > 0; }

private:
    ShaderProgram mShaderProgram;
    GLuint mVao{0};
    GLuint mVbo{0};
    GLsizei mVertexCount{0};
    GLint mMvpUniformLocation{-1};

    // Infill geometry (separate VAO for toggling)
    GLuint mInfillVao{0};
    GLuint mInfillVbo{0};
    GLsizei mInfillVertexCount{0};

    // Travel moves geometry (separate VAO for toggling)
    GLuint mTravelVao{0};
    GLuint mTravelVbo{0};
    GLsizei mTravelVertexCount{0};

    // TCP orientation arrows (one arrow per non-travel segment)
    GLuint mOrientationVao{0};
    GLuint mOrientationVbo{0};
    GLsizei mOrientationVertexCount{0};

    // Origin marker
    GLuint mOriginVao{0};
    GLuint mOriginVbo{0};
    GLsizei mOriginVertexCount{0};

    /**
     * @brief Returns a color for a branch ID from the colorblind-friendly palette.
     * @param branchId Branch identifier (cycles for > 8 branches).
     * @return RGB color.
     */
    static glm::vec3 branchColor(uint32_t branchId);
};

#endif // MEDUSA_SRC_GRAPHICS_TOOLPATH_RENDERER_H_
