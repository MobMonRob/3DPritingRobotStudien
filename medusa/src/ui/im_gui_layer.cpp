/**
 * @file im_gui_layer.cpp
 * @brief Implementation of ImGuiLayer methods for Dear ImGui context and frame management.
 */
#include "im_gui_layer.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "imgui.h"
#include "logger.h"
#include "ui_style.h"

bool ImGuiLayer::init(GLFWwindow* window)
{
    if (!window)
    {
        MEDUSA_ERROR("ImGuiLayer init failed: window is null");
        mReady = false;
        return false;
    }
    MEDUSA_INFO("Initializing ImGui");
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    MEDUSA_DEBUG("ImGui config: keyboardNav={}, docking={}, viewports={}",
                 (io.ConfigFlags & ImGuiConfigFlags_NavEnableKeyboard) != 0,
                 (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) != 0,
                 (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0);

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
    {
        MEDUSA_CRITICAL("ImGui GLFW backend initialization failed");
        ImGui::DestroyContext();
        mReady = false;
        return false;
    }

#ifdef __APPLE__
    auto glsl_version = "#version 150";
#else
    auto glsl_version = "#version 130";
#endif

    if (!ImGui_ImplOpenGL3_Init(glsl_version))
    {
        MEDUSA_CRITICAL("ImGui OpenGL3 backend initialization failed (GLSL={})", glsl_version);
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        mReady = false;
        return false;
    }

    mReady = true;
    MEDUSA_INFO("ImGui initialized");
    return true;
}

void ImGuiLayer::shutdown()
{
    if (!mReady)
    {
        MEDUSA_DEBUG("ImGuiLayer shutdown skipped (not initialized)");
        return;
    }

    MEDUSA_INFO("Shutting down ImGui");

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    mReady = false;
}

bool ImGuiLayer::isReady() const
{
    return mReady;
}

void ImGuiLayer::beginFrame() const
{
    if (!mReady)
    {
        return;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::endFrameAndRender() const
{
    if (!mReady)
    {
        return;
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    const ImGuiIO& io = ImGui::GetIO();

    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        GLFWwindow* previous_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(previous_context);
    }
}

void ImGuiLayer::drawDockspaceHost()
{
    const ImGuiIO& io = ImGui::GetIO();
    const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(main_viewport->Pos);
    ImGui::SetNextWindowSize(main_viewport->Size);
    ImGui::SetNextWindowViewport(main_viewport->ID);

    constexpr ImGuiWindowFlags kHostFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("DockSpaceRoot", nullptr, kHostFlags | ImGuiWindowFlags_MenuBar);
    ImGui::PopStyleVar(3);

    if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
    {
        const ImGuiID kDockspaceId = ImGui::GetID("MainDockSpace");
        ImGui::DockSpace(kDockspaceId, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);
    }

    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Files", nullptr, nullptr);
            ImGui::MenuItem("Settings", nullptr, nullptr);
            ImGui::Separator();
            if (ImGui::MenuItem("Reset Layout"))
            {
                MEDUSA_INFO("Layout reset requested");
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Theme"))
        {
            if (ImGui::MenuItem("Medusa (Dark)"))
            {
                MEDUSA_INFO("ImGui theme changed: Medusa Dark");
                ui::applyMedusaStyle();
            }

            if (ImGui::MenuItem("Medusa (Light)"))
            {
                MEDUSA_INFO("ImGui theme changed: Medusa Light");
                ui::applyLightTheme();
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Dark"))
            {
                MEDUSA_INFO("ImGui theme changed: Dark");
                ImGui::StyleColorsDark();
            }

            if (ImGui::MenuItem("Light"))
            {
                MEDUSA_INFO("ImGui theme changed: Light");
                ImGui::StyleColorsLight();
            }

            if (ImGui::MenuItem("Classic"))
            {
                MEDUSA_INFO("ImGui theme changed: Classic");
                ImGui::StyleColorsClassic();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help"))
        {
            ImGui::TextDisabled("Medusa 3D Viewer");
            ImGui::Separator();
            ImGui::TextDisabled("Controls:");
            ImGui::BulletText("Left-drag: Rotate model");
            ImGui::BulletText("Right-drag: Orbit camera");
            ImGui::BulletText("Scroll: Zoom");
            ImGui::BulletText("ESC: Quit");
            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }
    ImGui::End();
}