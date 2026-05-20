/**
 * @file scene_renderer.cpp
 * @brief Implementation of @ref SceneRenderer.
 */
#include <glm/gtc/type_ptr.hpp>

#include "camera.h"
#include "logger.h"
#include "mesh.h"
#include "scene_renderer.h"


// Shader: Lambert-style shading with optional wireframe color.
static auto kSceneVsSrc = R"GLSL(
    #version 150
    in vec3 aPos;
    in vec3 aNormal;
    uniform mat4 uMVP;
    uniform mat3 uNormalMat;
    out vec3 vNormal;
    void main() {
        vNormal = normalize(uNormalMat * aNormal);
        gl_Position = uMVP * vec4(aPos, 1.0);
    }
)GLSL";

static auto kSceneFsSrc = R"GLSL(
    #version 150
    in vec3 vNormal;
    uniform bool uWireframe;
    out vec4 FragColor;
    void main() {
        vec3 base = vec3(0.20, 0.55, 0.90);
        if (uWireframe) {
            FragColor = vec4(base, 1.0);
        } else {
            vec3 N = normalize(vNormal);
            vec3 L = normalize(vec3(0.5, 1.0, 0.3));
            float diff = max(dot(N, L), 0.0);
            vec3 color = base * (0.25 + 0.75 * diff);
            FragColor = vec4(color, 1.0);
        }
    }
)GLSL";

bool SceneRenderer::initialize()
{
    MEDUSA_INFO("Initializing SceneRenderer shaders");

    if (!mShaderProgram.create(kSceneVsSrc, kSceneFsSrc))
    {
        MEDUSA_CRITICAL("SceneRenderer shader program creation failed");
        return false;
    }

    mMvpUniformLocation = mShaderProgram.loc("uMVP");
    mNormalMatrixUniformLocation = mShaderProgram.loc("uNormalMat");
    mWireframeUniformLocation = mShaderProgram.loc("uWireframe");

    MEDUSA_DEBUG("SceneRenderer uniforms: uMVP={}, uNormalMat={}, uWireframe={}", mMvpUniformLocation,
                 mNormalMatrixUniformLocation, mWireframeUniformLocation);

    if (mMvpUniformLocation < 0 || mNormalMatrixUniformLocation < 0 || mWireframeUniformLocation < 0)
    {
        MEDUSA_WARN("SceneRenderer: one or more uniform locations are invalid (shader optimized?)");
    }

    return true;
}

void SceneRenderer::render(GLFWwindow* window,
                           const Camera& camera,
                           const graphics::Mesh& mesh,
                           const glm::mat4& modelMatrix,
                           const bool renderWireframe) const
{
    if (!window)
    {
        MEDUSA_WARN("SceneRenderer::render called with null window");
        return;
    }

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);

    if (framebufferWidth <= 0 || framebufferHeight <= 0)
    {
        MEDUSA_DEBUG("Framebuffer size is invalid ({}x{}); skipping render", framebufferWidth, framebufferHeight);
        return;
    }

    glViewport(0, 0, framebufferWidth, framebufferHeight);
    glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // If no mesh is loaded, keep background only.
    if (mesh.vao == 0 || mesh.count == 0)
    {
        static bool s_loggedEmptyMeshOnce = false;
        if (!s_loggedEmptyMeshOnce)
        {
            MEDUSA_DEBUG("No mesh data present yet (vao=0 or count=0). Rendering background only.");
            s_loggedEmptyMeshOnce = true;
        }
        return;
    }

    const float kAspect = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);

    const glm::mat4 kProj = camera.proj(kAspect);
    const glm::mat4 kView = camera.view();

    const glm::mat4 kMvp = kProj * kView * modelMatrix;

    // Note: modelMatrix uses uniform scale, so mat3(modelMatrix) is sufficient here.
    const glm::mat3 kNormalMatrix(modelMatrix);

    mShaderProgram.use();
    glUniformMatrix4fv(mMvpUniformLocation, 1, GL_FALSE, glm::value_ptr(kMvp));
    glUniformMatrix3fv(mNormalMatrixUniformLocation, 1, GL_FALSE, glm::value_ptr(kNormalMatrix));

    glBindVertexArray(mesh.vao);
    if (renderWireframe)
    {
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(2.0f);
        glUniform1i(mWireframeUniformLocation, 1);
        glDrawArrays(GL_TRIANGLES, 0, mesh.count);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    else
    {
        glUniform1i(mWireframeUniformLocation, 0);
        glDrawArrays(GL_TRIANGLES, 0, mesh.count);
    }
    glBindVertexArray(0);
}