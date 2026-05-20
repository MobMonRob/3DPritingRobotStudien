#include "imgui.h"

#include <glad/glad.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <glm/glm.hpp>

#include "app.h"
#include "logger.h"
#include "ui_style.h"

int App::run(const int argc, char** argv)
{
    MEDUSA_INFO("Application starting");
    if (!initGL())
    {
        MEDUSA_CRITICAL("OpenGL initialization failed - exiting");
        return 1;
    }
    MEDUSA_DEBUG("Initializing SceneRenderer");

    if (!mSceneRenderer.initialize())
    {
        MEDUSA_CRITICAL("SceneRenderer initialization failed - exiting");
        return 1;
    }
    MEDUSA_DEBUG("Initializing ImGuiLayer");

    if (!mImGuiLayer.init(mWindow))
    {
        MEDUSA_CRITICAL("ImGuiLayer initialization failed - exiting");
        return 1;
    }

    ui::applyMedusaStyle();
    MEDUSA_DEBUG("Initializing UIRenderer");

    if (!mUIRenderer.init())
    {
        MEDUSA_CRITICAL("UIRenderer initialization failed - exiting");
        return 1;
    }
    MEDUSA_DEBUG("Initializing AxesRenderer");

    if (!mAxesRenderer.initialize())
    {
        MEDUSA_CRITICAL("AxesRenderer initialization failed - exiting");
        return 1;
    }
    MEDUSA_DEBUG("Initializing GridRenderer");

    if (!mGridRenderer.initialize())
    {
        MEDUSA_CRITICAL("GridRenderer initialization failed - exiting");
        return 1;
    }

    scanSampleFiles();
    if (argc > 1)
    {
        MEDUSA_INFO("Command line file specified: {}", argv[1]);
        loadMesh(argv[1]);
    }

    mStartTimeSeconds = glfwGetTime();
    loop();
    MEDUSA_INFO("Shutting down");
    mImGuiLayer.shutdown();
    mUIRenderer.shutdown();
    shutdownGL();
    MEDUSA_INFO("Shutdown complete");
    return 0;
}

bool App::initGL()
{
    MEDUSA_DEBUG("Initializing GLFW");
    if (!glfwInit())
    {
        MEDUSA_CRITICAL("GLFW initialization failed");
        return false;
    }

#ifdef __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    mWindow = glfwCreateWindow(800, 600, "Medusa", nullptr, nullptr);
    if (!mWindow)
    {
        MEDUSA_CRITICAL("Failed to create GLFW window");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(mWindow);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(mWindow, this);
    glfwSetScrollCallback(mWindow, [](GLFWwindow* w, double /*xoff*/, const double yoff)
                          {
                              auto* self = static_cast<App*>(glfwGetWindowUserPointer(w));
                              if (!self)
                              {
                                  return;
                              }

                              if (ImGui::GetCurrentContext())
                              {
                                  if (const ImGuiIO& io = ImGui::GetIO(); io.WantCaptureMouse)
                                      return;
                              }

                              self->mCameraController.updateFromScroll(yoff);
                          }
    );

    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0)
    {
        MEDUSA_CRITICAL("gladLoadGLLoader failed");
        cleanupWindow();
        glfwTerminate();
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    const GLubyte* gl_version = glGetString(GL_VERSION);
    const GLubyte* gl_renderer = glGetString(GL_RENDERER);

    if (gl_version && gl_renderer)
    {
        MEDUSA_INFO("OpenGL initialized: {} ({})",
                    reinterpret_cast<const char*>(gl_version),
                    reinterpret_cast<const char*>(gl_renderer));
    }
    else
    {
        MEDUSA_INFO("OpenGL initialized");
    }

    return true;
}

void App::shutdownGL()
{
    MEDUSA_DEBUG("Shutting down GLFW/OpenGL");
    cleanupWindow();
    glfwTerminate();
}

void App::cleanupWindow()
{
    if (mWindow)
    {
        MEDUSA_DEBUG("Destroying window");
        glfwDestroyWindow(mWindow);
        mWindow = nullptr;
    }
}

void App::scanSampleFiles()
{
    static auto kSampleFilesPath = MEDUSA_PROJECT_ROOT
    "/data/samplefiles";
    MEDUSA_INFO("Scanning for mesh files: {}", kSampleFilesPath);
    mMeshFileBrowser.scanDirectory(kSampleFilesPath);
}

bool App::loadMesh(const std::string& path)
{
    MEDUSA_INFO("Loading mesh: {}", path);

    if (const std::vector kCands{path}; !mMesh.loadFromCandidates(kCands))
    {
        mLoadStatus = "Failed to load file";
        mLoadedFileName.clear();
        MEDUSA_ERROR("Failed to load mesh: {}", path);
        return false;
    }

    // Extract just the filename for display
    const std::filesystem::path fsPath(path);
    mLoadedFileName = fsPath.filename().string();
    mLoadStatus = "Loaded successfully";
    MEDUSA_INFO("Mesh loaded: {}", path);
    return true;
}

void App::handleCloseHotkeys() const
{
    if (glfwGetKey(mWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(mWindow, GL_TRUE);
    }
}

void App::resetManualRotation()
{
    MEDUSA_INFO("Resetting manual rotation");
    mModelController.getTransform().resetManualRotation();
}

void App::updateManualRotationFromMouseDrag()
{
    const ImGuiIO& io = ImGui::GetIO();
    mModelController.updateFromMouseDrag(mWindow, io.WantCaptureMouse);
    mCameraController.updateFromMouseDrag(mWindow, io.WantCaptureMouse);
}

void App::renderScene3d() const
{
    const auto kElapsedSeconds = static_cast<float>(glfwGetTime() - mStartTimeSeconds);
    const glm::mat4 kModel = mModelController.getTransform().computeModelMatrix(kElapsedSeconds, mMesh);
    mSceneRenderer.render(mWindow, mCameraController.getCamera(), mMesh, kModel, mWireframe);

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(mWindow, &framebufferWidth, &framebufferHeight);

    if (framebufferWidth > 0 && framebufferHeight > 0)
    {
        const float kAspect = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
        // Use rotation-only matrix for helpers (no mesh-specific translation/scale)
        const glm::mat4 kHelperMatrix = mModelController.getTransform().computeRotationMatrix(kElapsedSeconds);

        if (mShowGrid)
        {
            mGridRenderer.render(mCameraController.getCamera(), kHelperMatrix, kAspect);
        }

        if (mShowAxes)
        {
            mAxesRenderer.render(mCameraController.getCamera(), kHelperMatrix, kAspect);
        }
    }
}

void App::drawFilesWindow()
{
    mMeshFileBrowser.setStatus(mLoadStatus);
    mMeshFileBrowser.draw("Files", [this](const std::string& fullPath) { loadMesh(fullPath); });
}

void App::drawSettingsWindow()
{
    ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Settings"))
    {
        ImGui::End();
        return;
    }

    // --- Model Info Section ---
    if (ImGui::CollapsingHeader("Model", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (!mLoadedFileName.empty())
        {
            ImGui::Text("File: %s", mLoadedFileName.c_str());
            ImGui::Spacing();

            // Mesh statistics
            ImGui::TextDisabled("Statistics");
            ImGui::Separator();

            const int vertexCount = mMesh.count;
            const int triangleCount = vertexCount / 3;

            ImGui::Text("Vertices:  %d", vertexCount);
            ImGui::Text("Triangles: %d", triangleCount);

            ImGui::Spacing();

            // Bounding box info
            ImGui::TextDisabled("Dimensions");
            ImGui::Separator();

            const glm::vec3& center = mMesh.center;
            const float radius = mMesh.radius;
            const float diameter = radius * 2.0f;

            ImGui::Text("Size:   %.2f units", diameter);
            ImGui::Text("Center: (%.2f, %.2f, %.2f)", center.x, center.y, center.z);
        }
        else
        {
            ImGui::TextDisabled("No model loaded");
            ImGui::TextDisabled("Select a file from the browser");
        }
    }

    ImGui::Spacing();

    // --- Display Section ---
    if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::TextDisabled("Render Mode");
        ImGui::Separator();

        ImGui::Checkbox("Wireframe", &mWireframe);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Show triangle edges instead of filled faces");
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Scene Helpers");
        ImGui::Separator();

        ImGui::Checkbox("Show Axes", &mShowAxes);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("X = Red, Y = Green, Z = Blue");
        }

        ImGui::Checkbox("Show Grid", &mShowGrid);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Ground plane reference grid");
        }
    }

    ImGui::Spacing();

    // --- Animation Section ---
    if (ImGui::CollapsingHeader("Animation", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Auto-rotate", &mModelController.getTransform().isAutoRotateEnabled);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Continuously rotate the model around Y-axis");
        }

        if (!mModelController.getTransform().isAutoRotateEnabled)
        {
            ImGui::Spacing();
            if (ImGui::Button("Reset Rotation", ImVec2(-1, 0)))
            {
                resetManualRotation();
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Reset to default view orientation");
            }
        }
    }

    ImGui::End();
}

void App::renderImGui()
{
    if (!mImGuiLayer.isReady())
    {
        return;
    }

    mImGuiLayer.beginFrame();
    ImGuiLayer::drawDockspaceHost();
    drawFilesWindow();
    drawSettingsWindow();
    mImGuiLayer.endFrameAndRender();
}

void App::loop()
{
    MEDUSA_INFO("Entering main loop");
    while (!glfwWindowShouldClose(mWindow))
    {
        glfwPollEvents();
        handleCloseHotkeys();
        updateManualRotationFromMouseDrag();
        renderScene3d();
        renderImGui();
        glfwSwapBuffers(mWindow);
    }
    MEDUSA_INFO("Main loop exited");
}