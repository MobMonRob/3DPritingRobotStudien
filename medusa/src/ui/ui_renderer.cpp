/**
 * @file ui_renderer.cpp
 * @brief Implementation of UIRenderer methods for simple 2D UI rendering.
 */
#include "ui_renderer.h"

#include <array>

#include "logger.h"

namespace
{
    constexpr auto kVertexShaderSource = R"GLSL(
    #version 150
    in vec2 aPos;
    void main() { gl_Position = vec4(aPos, 0.0, 1.0); }
)GLSL";
    constexpr auto kFragmentShaderSource = R"GLSL(
    #version 150
    uniform vec3 uColor;
    out vec4 FragColor;
    void main() { FragColor = vec4(uColor, 1.0); }
)GLSL";
} // namespace

bool UIRenderer::init()
{
    MEDUSA_INFO("UIRenderer: Initializing shader program and GPU buffers");
    if (!shaderProgram.create(kVertexShaderSource, kFragmentShaderSource))
    {
        MEDUSA_ERROR("UIRenderer: Failed to create shader program");
        return false;
    }
    colorUniformLocation = shaderProgram.loc("uColor");
    if (colorUniformLocation < 0)
    {
        MEDUSA_WARN("UIRenderer: Uniform 'uColor' not found in shader program");
    }
    glGenVertexArrays(1, &uiVao);
    glGenBuffers(1, &uiVbo);
    glBindVertexArray(uiVao);
    glBindBuffer(GL_ARRAY_BUFFER, uiVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 12, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    glBindVertexArray(0);
    MEDUSA_DEBUG("UIRenderer: Initialization complete (VAO={}, VBO={}, programId={})", uiVao, uiVbo,
                 shaderProgram.programId);
    return true;
}

void UIRenderer::shutdown() const
{
    MEDUSA_INFO("UIRenderer: Releasing GPU resources");
    if (uiVbo)
        glDeleteBuffers(1, &uiVbo);
    if (uiVao)
        glDeleteVertexArrays(1, &uiVao);
}

bool UIRenderer::button(GLFWwindow* window, int posX, int posY, int width, int height, const glm::vec3& colorInactive,
                        const glm::vec3& colorActive, bool isActive)
{
    double mouseX = 0.0, mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    const bool kIsHovered = (mouseX >= posX && mouseX <= posX + width && mouseY >= posY && mouseY <= posY + height);
    const bool kIsMouseDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool kWasClicked = kIsHovered && kIsMouseDown && !wasMouseDownLastFrame;
    wasMouseDownLastFrame = kIsMouseDown;
    if (kWasClicked)
    {
        MEDUSA_INFO("UIRenderer: Button clicked at ({},{}), size=({},{}), state={}", posX, posY, width, height,
                    isActive);
    }
    int framebufferWidth = 0, framebufferHeight = 0;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    const float kX0 = 2.0f * static_cast<float>(posX) / framebufferWidth - 1.0f;
    const float kX1 = 2.0f * static_cast<float>(posX + width) / framebufferWidth - 1.0f;
    const float kY0 = 1.0f - 2.0f * static_cast<float>(posY) / framebufferHeight;
    const float kY1 = 1.0f - 2.0f * static_cast<float>(posY + height) / framebufferHeight;
    const std::array kVertices = {kX0, kY1, kX1, kY1, kX1, kY0, kX0, kY1, kX1, kY0, kX0, kY0};
    glDisable(GL_DEPTH_TEST);
    shaderProgram.use();
    glBindVertexArray(uiVao);
    glBindBuffer(GL_ARRAY_BUFFER, uiVbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(kVertices), kVertices.data());
    const glm::vec3 kBaseColor = isActive ? colorActive : colorInactive;
    const glm::vec3 kHighlightColor = isActive ? glm::vec3(0.10f, 0.80f, 0.30f) : glm::vec3(0.25f, 0.45f, 0.95f);
    const glm::vec3 kFinalColor = kIsHovered ? kHighlightColor : kBaseColor;
    if (colorUniformLocation >= 0)
        glUniform3f(colorUniformLocation, kFinalColor.r, kFinalColor.g, kFinalColor.b);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
    if (kIsHovered)
    {
        MEDUSA_DEBUG("UIRenderer: Button hovered at ({},{}), state={}", posX, posY, isActive);
    }
    return kWasClicked;
}