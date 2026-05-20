#include "imgui.h"

#include <glad/glad.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "app.h"
#include "logger.h"
#include "ui_style.h"
#include "mesh_loader.h"
#include "toolpath.h"
#include "waypoint.h"
#include "serializer.h"

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

    MEDUSA_DEBUG("Initializing ToolpathRenderer");
    if (!mToolpathRenderer.initialize())
    {
        MEDUSA_CRITICAL("ToolpathRenderer initialization failed - exiting");
        return 1;
    }

    MEDUSA_DEBUG("Initializing PhiOverlayRenderer");
    if (!mPhiOverlay.initialize())
    {
        MEDUSA_CRITICAL("PhiOverlayRenderer initialization failed - exiting");
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

    // Load via Assimp into MeshLoadResult
    auto loadResult = graphics::MeshLoader::load(path, mZUpToYUp);
    if (!loadResult.isValid())
    {
        mLoadStatus = "Failed to load file";
        mLoadedFileName.clear();
        MEDUSA_ERROR("Failed to load mesh: {}", path);
        return false;
    }

    // Upload to GPU
    mMesh.build(loadResult.vertices, loadResult.bounds.center(), loadResult.bounds.min, loadResult.bounds.radius());

    // Convert to algorithmic TriangleMesh for slicing
    mTriangleMesh = geometry::convertFromInterleavedData(loadResult.vertices);

    // Clear previous slicing results
    mToolpathRenderer.clear();
    mPipelineResult = {};

    // Extract just the filename for display
    const std::filesystem::path fsPath(path);
    mLoadedFileName = fsPath.filename().string();
    mLoadedFilePath = path;
    mLoadStatus = "Loaded successfully";
    MEDUSA_DEBUG("Mesh stats: {} vertices, bounds radius={:.4f}",
                 loadResult.vertexCount(), loadResult.bounds.radius());
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

    // If mesh transparency is enabled and we have a toolpath, enable blending
    if (mMeshTransparent && mToolpathRenderer.hasData())
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    mSceneRenderer.render(mWindow, mCameraController.getCamera(), mMesh, kModel, mWireframe,
                          mMeshTransparent && mToolpathRenderer.hasData() ? 0.3f : 1.0f);

    if (mMeshTransparent && mToolpathRenderer.hasData())
    {
        glDisable(GL_BLEND);
    }

    // Phi heatmap overlay (Crystal step-2 validation). Drawn on top of the
    // shaded mesh, slightly polygon-offset so it isn't z-fought; uses the same
    // model matrix so it aligns 1:1 with the rendered geometry.
    if (mShowPhiOverlay && mPhiOverlay.hasData())
    {
        const auto& cam = mCameraController.getCamera();
        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(mWindow, &fbW, &fbH);
        const float aspect = (fbH > 0) ? static_cast<float>(fbW) / static_cast<float>(fbH) : 1.0f;
        const glm::mat4 mvp = cam.proj(aspect) * cam.view() * kModel;

        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        mPhiOverlay.render(mvp);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

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

        // Render toolpath overlay
        if (mShowToolpath && mToolpathRenderer.hasData())
        {
            const auto& cam = mCameraController.getCamera();
            glm::mat4 proj = cam.proj(kAspect);
            glm::mat4 view = cam.view();
            glm::mat4 mvp = proj * view * kModel;
            mToolpathRenderer.render(mvp, mShowInfill, mShowTravel, mShowOrientations);
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
        // Import settings (always visible)
        ImGui::TextDisabled("Import");
        ImGui::Separator();

        bool prevZUpToYUp = mZUpToYUp;
        ImGui::Checkbox("Z-up to Y-up", &mZUpToYUp);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Convert Z-up (CAD/3D-print) to Y-up (OpenGL).\nDisable for models already in Y-up.");
        }
        if (mZUpToYUp != prevZUpToYUp && !mLoadedFileName.empty())
        {
            // Find the last loaded path and reload with new setting
            // mLoadStatus reflects last op; re-use stored file path via mLoadedFilePath
            loadMesh(mLoadedFilePath);
        }

        if (!mLoadedFileName.empty())
        {
            ImGui::Spacing();
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
    drawSlicerWindow();
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

void App::drawSlicerWindow()
{
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Slicer"))
    {
        ImGui::End();
        return;
    }

    // --- Algorithm Selection ---
    if (ImGui::CollapsingHeader("Slicer Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const char* algorithms[] = {"Planar", "Base", "Crystal"};
        ImGui::Combo("Algorithm", &mSelectedAlgorithm, algorithms, IM_ARRAYSIZE(algorithms));

        ImGui::Spacing();
        ImGui::Separator();

        // --- Common Parameters ---
        ImGui::TextDisabled("General");
        float* thickness = (mSelectedAlgorithm == 0)
                               ? &mPlanarParams.layer_thickness
                               : (mSelectedAlgorithm == 1)
                                   ? &mBaseParams.layer_thickness
                                   : &mCrystalParams.layer_thickness;

        ImGui::SliderFloat("Layer Thickness", thickness,
                           slicing::SlicerParams::LAYER_THICKNESS_MIN,
                           slicing::SlicerParams::LAYER_THICKNESS_MAX, "%.3f mm");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::InputFloat("##thickness_input", thickness, 0.0f, 0.0f, "%.3f");
        *thickness = std::clamp(*thickness,
                                slicing::SlicerParams::LAYER_THICKNESS_MIN,
                                slicing::SlicerParams::LAYER_THICKNESS_MAX);

        // --- Base-specific Parameters ---
        if (mSelectedAlgorithm == 1)
        {
            ImGui::Spacing();
            ImGui::TextDisabled("Base Parameters");
            ImGui::Separator();

            ImGui::SliderFloat("Overhang Angle", &mBaseParams.overhang_angle,
                               slicing::BaseParams::OVERHANG_ANGLE_MIN,
                               slicing::BaseParams::OVERHANG_ANGLE_MAX, "%.1f deg");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            ImGui::InputFloat("##overhang_input", &mBaseParams.overhang_angle, 0.0f, 0.0f, "%.1f");
            mBaseParams.overhang_angle = std::clamp(mBaseParams.overhang_angle,
                                                     slicing::BaseParams::OVERHANG_ANGLE_MIN,
                                                     slicing::BaseParams::OVERHANG_ANGLE_MAX);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Max overhang angle before creating a virtual wall (0 = no overhang, 89 = very permissive)");
            }

            ImGui::SliderInt("Max Iterations", &mBaseParams.max_iterations,
                             slicing::BaseParams::MAX_ITERATIONS_MIN,
                             slicing::BaseParams::MAX_ITERATIONS_MAX);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Safety limit for the number of queue-based slicing runs");
            }
        }

        // --- Crystal-specific Parameters ---
        if (mSelectedAlgorithm == 2)
        {
            ImGui::Spacing();
            ImGui::TextDisabled("Crystal Parameters");
            ImGui::Separator();

            ImGui::SliderInt("Planar Base Layers", &mCrystalParams.planar_base_layers,
                             slicing::crystal::CrystalParams::PLANAR_BASE_LAYERS_MIN,
                             slicing::crystal::CrystalParams::PLANAR_BASE_LAYERS_MAX);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Number of initial planar Z-layers for bed adhesion");
            }

            ImGui::SliderFloat("Infill Spacing", &mCrystalParams.infill_spacing,
                               slicing::crystal::CrystalParams::INFILL_SPACING_MIN,
                               slicing::crystal::CrystalParams::INFILL_SPACING_MAX, "%.2f mm");

            ImGui::Checkbox("Adaptive Thickness", &mCrystalParams.adaptive_thickness);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Phase-2 feature; ignored by the current implementation");
            }
        }

        ImGui::Spacing();

        // --- Start Button ---
        bool canSlice = mTriangleMesh.isValid();
        if (!canSlice)
        {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Start Slicing", ImVec2(-1, 30)))
        {
            runSlicing();
        }
        if (!canSlice)
        {
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            {
                ImGui::SetTooltip("Load a mesh first");
            }
        }
    }

    ImGui::Spacing();

    // --- Status Section ---
    if (mPipelineResult.success && ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen))
    {
        float progress = mPipelineResult.coverage;
        ImGui::ProgressBar(progress, ImVec2(-1, 0));
        ImGui::Text("Coverage: %.1f%%", progress * 100.0f);
        ImGui::Text("Segments: %u", mPipelineResult.toolpath.num_segments());
        ImGui::Text("Branches: %u", mPipelineResult.toolpath.num_branches());
        ImGui::Text("Length:   %.1f mm", mPipelineResult.toolpath.total_length());
        ImGui::Text("Layers:   %zu", mPipelineResult.layers.size());

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled("Display");

        ImGui::Checkbox("Show Toolpath", &mShowToolpath);
        ImGui::Checkbox("Show Infill", &mShowInfill);
        ImGui::Checkbox("Show Travel Moves", &mShowTravel);
        ImGui::Checkbox("Mesh Transparency", &mMeshTransparent);
        ImGui::Checkbox("Show Orientations", &mShowOrientations);

        // Crystal scalar-field overlay. Toggle is always visible but only
        // takes effect once a Crystal run has produced a Phi field.
        ImGui::BeginDisabled(!mPhiOverlay.hasData());
        ImGui::Checkbox("Phi Heatmap (Crystal)", &mShowPhiOverlay);
        ImGui::EndDisabled();
        if (!mPhiOverlay.hasData() && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        {
            ImGui::SetTooltip("Run Crystal to compute the harmonic field");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled("Kronos Export");

        if (!mExportOutputDir.empty())
        {
            ImGui::Text("Dir: %s", mExportOutputDir.c_str());
        }

        if (!mExportStatus.empty())
        {
            if (mExportStatus == "OK")
            {
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "Export OK");
                if (!mLastExportPath.empty())
                {
                    ImGui::SetNextItemWidth(-1);
                    ImGui::TextWrapped("%s", mLastExportPath.c_str());
                }
            }
            else
            {
                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Export failed");
                ImGui::TextWrapped("%s", mExportStatus.c_str());
            }
        }
        else
        {
            ImGui::TextDisabled("No export yet");
        }
    }

    ImGui::End();
}

void App::runSlicing()
{
    static const char* algoNames[] = {"Planar", "Base", "Crystal"};
    MEDUSA_INFO("Running slicing pipeline (algorithm: {})", algoNames[mSelectedAlgorithm]);

    if (mSelectedAlgorithm == 0)
    {
        MEDUSA_DEBUG("PlanarSlicer params: layer_thickness={:.3f}, up_axis={}",
                     mPlanarParams.layer_thickness, mPlanarParams.up_axis);
        mSlicer = std::make_unique<slicing::PlanarSlicer>(mPlanarParams);
    }
    else if (mSelectedAlgorithm == 1)
    {
        MEDUSA_DEBUG("BaseSlicer params: layer_thickness={:.3f}, overhang_angle={:.1f}, max_iterations={}",
                     mBaseParams.layer_thickness, mBaseParams.overhang_angle, mBaseParams.max_iterations);
        mSlicer = std::make_unique<slicing::BaseSlicer>(mBaseParams);
    }
    else
    {
        MEDUSA_DEBUG("CrystalSlicer params: layer_thickness={:.3f}, planar_base={}, walls={}, infill_spacing={:.2f}",
                     mCrystalParams.layer_thickness, mCrystalParams.planar_base_layers,
                     mCrystalParams.wall_count, mCrystalParams.infill_spacing);
        mSlicer = std::make_unique<slicing::crystal::CrystalSlicer>(mCrystalParams);
    }

    mPipelineResult = slicing::SlicingPipeline::execute(*mSlicer, mTriangleMesh);

    // Crystal exposes the harmonic scalar field for visual validation.
    if (auto* fs = dynamic_cast<slicing::crystal::CrystalSlicer*>(mSlicer.get()))
    {
        const auto& sf = fs->scalar_field();
        if (sf.valid)
        {
            // Use the refined mesh — sf.phi is indexed against it, not against
            // the original loader mesh.
            mPhiOverlay.upload(fs->refined_mesh(), sf.phi, sf.phi_min, sf.phi_max);
        }
        else
        {
            mPhiOverlay.clear();
        }
    }
    else
    {
        mPhiOverlay.clear();
    }

    if (mPipelineResult.success)
    {
        mToolpathRenderer.upload(mPipelineResult.toolpath);
        MEDUSA_INFO("Slicing complete: {} segments", mPipelineResult.toolpath.num_segments());
        exportToolpath();
    }
    else
    {
        mToolpathRenderer.clear();
        MEDUSA_WARN("Slicing produced no results");
    }
}

void App::exportToolpath()
{
    using namespace medusa::kronos;

    // --- Initialise output directory (once) ---
    if (mExportOutputDir.empty())
    {
        mExportOutputDir = std::filesystem::path(MEDUSA_PROJECT_ROOT) / "output";
    }

    // --- Map algorithm enum ---
    static const SlicerAlgorithm kAlgoMap[] = {
        SlicerAlgorithm::Planar,
        SlicerAlgorithm::Base,
        SlicerAlgorithm::Crystal
    };
    const SlicerAlgorithm algo = kAlgoMap[mSelectedAlgorithm];

    // --- Timestamp for created_at (ISO 8601 UTC) ---
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
    gmtime_r(&t, &utc_tm);
    char ts_buf[21];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);

    // --- Convert slicing::Toolpath → kronos::Toolpath ---
    // Each segment contributes one waypoint at its end point.
    // Before the first print segment the start point is inserted as a travel waypoint.
    Toolpath kronos_tp;
    kronos_tp.metadata.slicer_version  = "0.0.1";
    kronos_tp.metadata.algorithm       = algo;
    kronos_tp.metadata.reference_frame = "workpiece";
    kronos_tp.metadata.created_at      = ts_buf;

    const auto& segs = mPipelineResult.toolpath.segments;
    kronos_tp.waypoints.reserve(segs.size() + 1);

    const glm::vec3 kDefaultOri(0.0f, 0.0f, 1.0f);

    // Computes the quaternion that rotates (0,0,1) onto the given orientation vector.
    // Uses the half-vector method (no GLM GTX extension required).
    auto seg_to_quaternion = [&](const glm::vec3& ori) -> std::tuple<double,double,double,double>
    {
        const glm::vec3 from = kDefaultOri;
        const glm::vec3 to   = (glm::length(ori) > 1e-6f) ? glm::normalize(ori) : kDefaultOri;

        const float dot = glm::dot(from, to);

        // Opposing vectors: 180 deg rotation around any axis perpendicular to 'from'
        if (dot < -1.0f + 1e-6f)
        {
            // Axis perpendicular to (0,0,1): use (1,0,0)
            return {0.0, 1.0, 0.0, 0.0};
        }

        const glm::vec3 cross = glm::cross(from, to);
        const float     w     = 1.0f + dot; // = 2*cos^2(theta/2)
        const float     len   = std::sqrt(w * w + glm::dot(cross, cross));

        if (len < 1e-9f)
        {
            return {1.0, 0.0, 0.0, 0.0}; // identity quaternion
        }

        return {
            static_cast<double>(w     / len),
            static_cast<double>(cross.x / len),
            static_cast<double>(cross.y / len),
            static_cast<double>(cross.z / len)
        };
    };

    for (std::size_t i = 0; i < segs.size(); ++i)
    {
        const auto& seg = segs[i];

        // Add the start point of the very first segment as its own waypoint
        if (i == 0)
        {
            const auto [qw, qx, qy, qz] = seg_to_quaternion(seg.orientation);
            Waypoint wp_start;
            wp_start.x          = static_cast<double>(seg.start.x);
            wp_start.y          = static_cast<double>(seg.start.y);
            wp_start.z          = static_cast<double>(seg.start.z);
            wp_start.qw = qw; wp_start.qx = qx; wp_start.qy = qy; wp_start.qz = qz;
            wp_start.feed_rate  = seg.is_travel ? 100.0 : 50.0;
            wp_start.extrusion  = seg.is_travel ? 0.0   : 2.0;
            wp_start.motion_type = seg.is_travel ? MotionType::Travel : MotionType::Print;
            wp_start.layer_id   = seg.growth_step;
            wp_start.segment_id = static_cast<std::uint32_t>(i);
            wp_start.branch_id  = seg.branch_id;
            kronos_tp.waypoints.push_back(wp_start);
        }

        const auto [qw, qx, qy, qz] = seg_to_quaternion(seg.orientation);
        Waypoint wp;
        wp.x          = static_cast<double>(seg.end.x);
        wp.y          = static_cast<double>(seg.end.y);
        wp.z          = static_cast<double>(seg.end.z);
        wp.qw = qw; wp.qx = qx; wp.qy = qy; wp.qz = qz;
        wp.feed_rate  = seg.is_travel ? 100.0 : 50.0;
        wp.extrusion  = seg.is_travel ? 0.0   : 2.0;
        wp.motion_type = seg.is_travel ? MotionType::Travel : MotionType::Print;
        wp.layer_id   = seg.growth_step;
        wp.segment_id = static_cast<std::uint32_t>(i);
        wp.branch_id  = seg.branch_id;
        kronos_tp.waypoints.push_back(wp);
    }

    // --- Export ---
    KronosExporter exporter{mExportOutputDir};
    const auto result = exporter.export_job(kronos_tp);

    if (result.is_ok())
    {
        mLastExportPath = result.value().string();
        mExportStatus   = "OK";
        MEDUSA_INFO("Kronos export successful: {}", mLastExportPath);
    }
    else
    {
        mLastExportPath.clear();
        mExportStatus = result.error();
        MEDUSA_ERROR("Kronos export failed: {}", mExportStatus);
    }
}