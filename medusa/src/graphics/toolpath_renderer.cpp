/**
 * @file toolpath_renderer.cpp
 * @brief Implementation of the toolpath OpenGL renderer.
 */

#include "toolpath_renderer.h"
#include "common/toolpath.h"
#include "logger.h"

#include <glm/gtc/type_ptr.hpp>
#include <array>

ToolpathRenderer::~ToolpathRenderer()
{
    clear();
}

bool ToolpathRenderer::initialize()
{
    MEDUSA_INFO("Initializing ToolpathRenderer shaders");

    if (!mShaderProgram.createFromFiles(
            MEDUSA_PROJECT_ROOT "/assets/shaders/toolpath.vert",
            MEDUSA_PROJECT_ROOT "/assets/shaders/toolpath.frag"))
    {
        MEDUSA_CRITICAL("ToolpathRenderer shader program creation failed");
        return false;
    }

    mMvpUniformLocation = mShaderProgram.loc("uMVP");
    if (mMvpUniformLocation < 0)
    {
        MEDUSA_WARN("ToolpathRenderer: uMVP uniform not found");
    }

    return true;
}

void ToolpathRenderer::upload(const slicing::Toolpath& toolpath)
{
    clear();

    if (toolpath.segments.empty())
    {
        MEDUSA_DEBUG("ToolpathRenderer::upload called with empty toolpath — nothing to upload");
        return;
    }

    // Split vertex data into contour, infill, and travel
    std::vector<float> contourData;
    std::vector<float> infillData;
    std::vector<float> travelData;
    contourData.reserve(toolpath.segments.size() * 2 * 6);
    infillData.reserve(toolpath.segments.size() * 2 * 6);
    travelData.reserve(toolpath.segments.size() * 6);

    static constexpr glm::vec3 kInfillColor{1.0f, 0.92f, 0.016f}; // Yellow
    static constexpr glm::vec3 kTravelColor{0.0f, 0.8f, 0.8f};    // Cyan

    for (const auto& seg : toolpath.segments)
    {
        std::vector<float>* buf;
        glm::vec3 color;

        if (seg.is_travel)
        {
            buf = &travelData;
            color = kTravelColor;
        }
        else if (seg.is_infill)
        {
            buf = &infillData;
            color = kInfillColor;
        }
        else
        {
            buf = &contourData;
            color = branchColor(seg.branch_id);
        }

        // Start vertex
        buf->push_back(seg.start.x);
        buf->push_back(seg.start.y);
        buf->push_back(seg.start.z);
        buf->push_back(color.r);
        buf->push_back(color.g);
        buf->push_back(color.b);

        // End vertex
        buf->push_back(seg.end.x);
        buf->push_back(seg.end.y);
        buf->push_back(seg.end.z);
        buf->push_back(color.r);
        buf->push_back(color.g);
        buf->push_back(color.b);
    }

    auto uploadVAO = [](GLuint& vao, GLuint& vbo, const std::vector<float>& data)
    {
        if (data.empty()) return;

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(data.size() * sizeof(float)),
                     data.data(), GL_STATIC_DRAW);

        // Position attribute (location 0)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);

        // Color attribute (location 1)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(3 * sizeof(float)));

        glBindVertexArray(0);
    };

    mVertexCount = static_cast<GLsizei>(contourData.size() / 6);
    uploadVAO(mVao, mVbo, contourData);

    mInfillVertexCount = static_cast<GLsizei>(infillData.size() / 6);
    uploadVAO(mInfillVao, mInfillVbo, infillData);

    mTravelVertexCount = static_cast<GLsizei>(travelData.size() / 6);
    uploadVAO(mTravelVao, mTravelVbo, travelData);

    // Build TCP orientation arrows: one short line per non-travel segment
    // pointing from the segment midpoint along the orientation vector.
    std::vector<float> orientationData;
    orientationData.reserve(toolpath.segments.size() * 2 * 6);
    static constexpr float          kOrientationScale = 2.0f;           // arrow length [mm]
    static constexpr glm::vec3      kOrientationColor{1.0f, 0.2f, 1.0f}; // magenta

    for (const auto& seg : toolpath.segments)
    {
        if (seg.is_travel) continue;
        const float len = glm::length(seg.orientation);
        if (len < 1e-6f) continue;

        const glm::vec3 mid = (seg.start + seg.end) * 0.5f;
        const glm::vec3 tip = mid + (seg.orientation / len) * kOrientationScale;

        orientationData.push_back(mid.x); orientationData.push_back(mid.y); orientationData.push_back(mid.z);
        orientationData.push_back(kOrientationColor.r); orientationData.push_back(kOrientationColor.g); orientationData.push_back(kOrientationColor.b);
        orientationData.push_back(tip.x); orientationData.push_back(tip.y); orientationData.push_back(tip.z);
        orientationData.push_back(kOrientationColor.r); orientationData.push_back(kOrientationColor.g); orientationData.push_back(kOrientationColor.b);
    }

    mOrientationVertexCount = static_cast<GLsizei>(orientationData.size() / 6);
    uploadVAO(mOrientationVao, mOrientationVbo, orientationData);

    // Upload origin marker (small cross)
    const glm::vec3& o = toolpath.origin;
    float s = 0.02f; // marker size
    glm::vec3 markerColor(1.0f, 1.0f, 1.0f); // white

    std::vector<float> originData = {
        // X axis
        o.x - s, o.y, o.z, markerColor.r, markerColor.g, markerColor.b,
        o.x + s, o.y, o.z, markerColor.r, markerColor.g, markerColor.b,
        // Y axis
        o.x, o.y - s, o.z, markerColor.r, markerColor.g, markerColor.b,
        o.x, o.y + s, o.z, markerColor.r, markerColor.g, markerColor.b,
        // Z axis
        o.x, o.y, o.z - s, markerColor.r, markerColor.g, markerColor.b,
        o.x, o.y, o.z + s, markerColor.r, markerColor.g, markerColor.b,
    };
    mOriginVertexCount = 6;

    glGenVertexArrays(1, &mOriginVao);
    glGenBuffers(1, &mOriginVbo);

    glBindVertexArray(mOriginVao);
    glBindBuffer(GL_ARRAY_BUFFER, mOriginVbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(originData.size() * sizeof(float)),
                 originData.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);

    MEDUSA_INFO("ToolpathRenderer: uploaded {} segments (contour={}, infill={}, travel={}, orientation={})",
                toolpath.segments.size(), mVertexCount, mInfillVertexCount,
                mTravelVertexCount, mOrientationVertexCount);
}

void ToolpathRenderer::render(const glm::mat4& mvp, bool showInfill, bool showTravel, bool showOrientations) const
{
    if (mVertexCount == 0 && mInfillVertexCount == 0 && mTravelVertexCount == 0) return;

    // Disable depth test so toolpath lines are always visible on top of the mesh
    glDisable(GL_DEPTH_TEST);

    mShaderProgram.use();
    glUniformMatrix4fv(mMvpUniformLocation, 1, GL_FALSE, glm::value_ptr(mvp));

    // Contour lines — thicker for clear visibility
    if (mVertexCount > 0)
    {
        glLineWidth(2.5f);
        glBindVertexArray(mVao);
        glDrawArrays(GL_LINES, 0, mVertexCount);
    }

    // Infill lines — medium weight
    if (showInfill && mInfillVertexCount > 0)
    {
        glLineWidth(1.5f);
        glBindVertexArray(mInfillVao);
        glDrawArrays(GL_LINES, 0, mInfillVertexCount);
    }

    // Travel moves — thin so print paths dominate visually
    if (showTravel && mTravelVertexCount > 0)
    {
        glLineWidth(1.0f);
        glBindVertexArray(mTravelVao);
        glDrawArrays(GL_LINES, 0, mTravelVertexCount);
    }

    // TCP orientation arrows (magenta)
    if (showOrientations && mOrientationVertexCount > 0)
    {
        glLineWidth(1.5f);
        glBindVertexArray(mOrientationVao);
        glDrawArrays(GL_LINES, 0, mOrientationVertexCount);
    }

    // Origin marker — thickest
    if (mOriginVertexCount > 0)
    {
        glLineWidth(4.0f);
        glBindVertexArray(mOriginVao);
        glDrawArrays(GL_LINES, 0, mOriginVertexCount);
    }

    glLineWidth(1.0f);
    glBindVertexArray(0);

    // Restore depth test
    glEnable(GL_DEPTH_TEST);
}

void ToolpathRenderer::clear()
{
    const bool hadData = (mVao != 0 || mInfillVao != 0 || mTravelVao != 0);
    if (mVao)
    {
        glDeleteVertexArrays(1, &mVao);
        mVao = 0;
    }
    if (mVbo)
    {
        glDeleteBuffers(1, &mVbo);
        mVbo = 0;
    }
    if (mInfillVao)
    {
        glDeleteVertexArrays(1, &mInfillVao);
        mInfillVao = 0;
    }
    if (mInfillVbo)
    {
        glDeleteBuffers(1, &mInfillVbo);
        mInfillVbo = 0;
    }
    if (mTravelVao)
    {
        glDeleteVertexArrays(1, &mTravelVao);
        mTravelVao = 0;
    }
    if (mTravelVbo)
    {
        glDeleteBuffers(1, &mTravelVbo);
        mTravelVbo = 0;
    }
    if (mOrientationVao)
    {
        glDeleteVertexArrays(1, &mOrientationVao);
        mOrientationVao = 0;
    }
    if (mOrientationVbo)
    {
        glDeleteBuffers(1, &mOrientationVbo);
        mOrientationVbo = 0;
    }
    if (mOriginVao)
    {
        glDeleteVertexArrays(1, &mOriginVao);
        mOriginVao = 0;
    }
    if (mOriginVbo)
    {
        glDeleteBuffers(1, &mOriginVbo);
        mOriginVbo = 0;
    }
    mVertexCount = 0;
    mInfillVertexCount = 0;
    mTravelVertexCount = 0;
    mOrientationVertexCount = 0;
    mOriginVertexCount = 0;
    if (hadData)
    {
        MEDUSA_DEBUG("ToolpathRenderer::clear: GPU buffers released");
    }
}

glm::vec3 ToolpathRenderer::branchColor(uint32_t branchId)
{
    // Colorblind-friendly palette (Wong, 2011)
    static constexpr std::array<glm::vec3, 8> palette = {{
        {1.0f,   1.0f,   1.0f},       // 0: White       #FFFFFF
        {0.902f, 0.624f, 0.0f},       // 1: Orange      #E69F00
        {0.337f, 0.706f, 0.914f},     // 2: Sky Blue    #56B4E9
        {0.0f,   0.620f, 0.451f},     // 3: Bluish Green #009E73
        {0.941f, 0.894f, 0.259f},     // 4: Yellow      #F0E442
        {0.0f,   0.447f, 0.698f},     // 5: Blue        #0072B2
        {0.835f, 0.369f, 0.0f},       // 6: Vermilion   #D55E00
        {0.800f, 0.475f, 0.655f},     // 7: Reddish Purple #CC79A7
    }};

    return palette[branchId % palette.size()];
}
