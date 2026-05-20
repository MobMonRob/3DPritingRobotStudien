/**
 * @file ui_style.cpp
 * @brief Implementation of custom ImGui styling.
 */

#include "ui_style.h"
#include "imgui.h"

namespace ui
{
    void applyMedusaStyle()
    {
        ImGuiStyle& style = ImGui::GetStyle();

        // Rounding
        style.WindowRounding = 6.0f;
        style.ChildRounding = 4.0f;
        style.FrameRounding = 4.0f;
        style.PopupRounding = 4.0f;
        style.ScrollbarRounding = 4.0f;
        style.GrabRounding = 4.0f;
        style.TabRounding = 4.0f;

        // Spacing
        style.WindowPadding = ImVec2(10.0f, 10.0f);
        style.FramePadding = ImVec2(8.0f, 4.0f);
        style.ItemSpacing = ImVec2(8.0f, 6.0f);
        style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
        style.IndentSpacing = 20.0f;
        style.ScrollbarSize = 14.0f;
        style.GrabMinSize = 12.0f;

        // Borders
        style.WindowBorderSize = 1.0f;
        style.ChildBorderSize = 1.0f;
        style.PopupBorderSize = 1.0f;
        style.FrameBorderSize = 0.0f;
        style.TabBorderSize = 0.0f;

        // Apply dark theme by default
        applyDarkTheme();
    }

    void applyDarkTheme()
    {
        ImVec4* colors = ImGui::GetStyle().Colors;

        // Background colors
        colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.11f, 0.13f, 1.00f);
        colors[ImGuiCol_ChildBg] = ImVec4(0.13f, 0.13f, 0.15f, 1.00f);
        colors[ImGuiCol_PopupBg] = ImVec4(0.11f, 0.11f, 0.13f, 0.96f);

        // Borders
        colors[ImGuiCol_Border] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);
        colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

        // Frame (input fields, checkboxes, etc.)
        colors[ImGuiCol_FrameBg] = ImVec4(0.18f, 0.18f, 0.21f, 1.00f);
        colors[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.24f, 0.28f, 1.00f);
        colors[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);

        // Title bar
        colors[ImGuiCol_TitleBg] = ImVec4(0.11f, 0.11f, 0.13f, 1.00f);
        colors[ImGuiCol_TitleBgActive] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.11f, 0.11f, 0.13f, 0.75f);

        // Menu bar
        colors[ImGuiCol_MenuBarBg] = ImVec4(0.13f, 0.13f, 0.15f, 1.00f);

        // Scrollbar
        colors[ImGuiCol_ScrollbarBg] = ImVec4(0.11f, 0.11f, 0.13f, 1.00f);
        colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);
        colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.35f, 0.35f, 0.40f, 1.00f);
        colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.42f, 0.42f, 0.48f, 1.00f);

        // Checkmark
        colors[ImGuiCol_CheckMark] = ImVec4(0.40f, 0.70f, 0.45f, 1.00f);

        // Slider
        colors[ImGuiCol_SliderGrab] = ImVec4(0.40f, 0.70f, 0.45f, 1.00f);
        colors[ImGuiCol_SliderGrabActive] = ImVec4(0.50f, 0.80f, 0.55f, 1.00f);

        // Button
        colors[ImGuiCol_Button] = ImVec4(0.22f, 0.22f, 0.26f, 1.00f);
        colors[ImGuiCol_ButtonHovered] = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
        colors[ImGuiCol_ButtonActive] = ImVec4(0.38f, 0.38f, 0.44f, 1.00f);

        // Header (collapsing headers, tree nodes, selectables)
        colors[ImGuiCol_Header] = ImVec4(0.22f, 0.22f, 0.26f, 1.00f);
        colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.45f, 0.32f, 1.00f);
        colors[ImGuiCol_HeaderActive] = ImVec4(0.32f, 0.55f, 0.38f, 1.00f);

        // Separator
        colors[ImGuiCol_Separator] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);
        colors[ImGuiCol_SeparatorHovered] = ImVec4(0.40f, 0.70f, 0.45f, 0.78f);
        colors[ImGuiCol_SeparatorActive] = ImVec4(0.40f, 0.70f, 0.45f, 1.00f);

        // Resize grip
        colors[ImGuiCol_ResizeGrip] = ImVec4(0.40f, 0.70f, 0.45f, 0.20f);
        colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.40f, 0.70f, 0.45f, 0.67f);
        colors[ImGuiCol_ResizeGripActive] = ImVec4(0.40f, 0.70f, 0.45f, 0.95f);

        // Tabs
        colors[ImGuiCol_Tab] = ImVec4(0.18f, 0.18f, 0.21f, 1.00f);
        colors[ImGuiCol_TabHovered] = ImVec4(0.28f, 0.45f, 0.32f, 1.00f);
        colors[ImGuiCol_TabSelected] = ImVec4(0.24f, 0.38f, 0.28f, 1.00f);
        colors[ImGuiCol_TabDimmed] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
        colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.20f, 0.32f, 0.24f, 1.00f);

        // Docking
        colors[ImGuiCol_DockingPreview] = ImVec4(0.40f, 0.70f, 0.45f, 0.70f);
        colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.11f, 0.11f, 0.13f, 1.00f);

        // Table
        colors[ImGuiCol_TableHeaderBg] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
        colors[ImGuiCol_TableBorderStrong] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);
        colors[ImGuiCol_TableBorderLight] = ImVec4(0.22f, 0.22f, 0.26f, 1.00f);
        colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
        colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.00f, 1.00f, 1.00f, 0.03f);

        // Text
        colors[ImGuiCol_Text] = ImVec4(0.92f, 0.92f, 0.94f, 1.00f);
        colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.52f, 1.00f);
        colors[ImGuiCol_TextSelectedBg] = ImVec4(0.40f, 0.70f, 0.45f, 0.35f);

        // Navigation
        colors[ImGuiCol_NavCursor] = ImVec4(0.40f, 0.70f, 0.45f, 1.00f);
        colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
        colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);

        // Modal dim
        colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.60f);
    }

    void applyLightTheme()
    {
        ImVec4* colors = ImGui::GetStyle().Colors;

        // Background colors
        colors[ImGuiCol_WindowBg] = ImVec4(0.96f, 0.96f, 0.97f, 1.00f);
        colors[ImGuiCol_ChildBg] = ImVec4(0.94f, 0.94f, 0.95f, 1.00f);
        colors[ImGuiCol_PopupBg] = ImVec4(0.98f, 0.98f, 0.98f, 0.98f);

        // Borders
        colors[ImGuiCol_Border] = ImVec4(0.78f, 0.78f, 0.80f, 1.00f);
        colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

        // Frame
        colors[ImGuiCol_FrameBg] = ImVec4(0.90f, 0.90f, 0.92f, 1.00f);
        colors[ImGuiCol_FrameBgHovered] = ImVec4(0.85f, 0.85f, 0.88f, 1.00f);
        colors[ImGuiCol_FrameBgActive] = ImVec4(0.80f, 0.80f, 0.84f, 1.00f);

        // Title bar
        colors[ImGuiCol_TitleBg] = ImVec4(0.90f, 0.90f, 0.92f, 1.00f);
        colors[ImGuiCol_TitleBgActive] = ImVec4(0.86f, 0.86f, 0.88f, 1.00f);
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.94f, 0.94f, 0.95f, 0.75f);

        // Menu bar
        colors[ImGuiCol_MenuBarBg] = ImVec4(0.92f, 0.92f, 0.94f, 1.00f);

        // Scrollbar
        colors[ImGuiCol_ScrollbarBg] = ImVec4(0.94f, 0.94f, 0.95f, 1.00f);
        colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.70f, 0.70f, 0.72f, 1.00f);
        colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.60f, 0.60f, 0.62f, 1.00f);
        colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.50f, 0.50f, 0.52f, 1.00f);

        // Checkmark
        colors[ImGuiCol_CheckMark] = ImVec4(0.20f, 0.55f, 0.30f, 1.00f);

        // Slider
        colors[ImGuiCol_SliderGrab] = ImVec4(0.20f, 0.55f, 0.30f, 1.00f);
        colors[ImGuiCol_SliderGrabActive] = ImVec4(0.25f, 0.65f, 0.35f, 1.00f);

        // Button
        colors[ImGuiCol_Button] = ImVec4(0.85f, 0.85f, 0.88f, 1.00f);
        colors[ImGuiCol_ButtonHovered] = ImVec4(0.78f, 0.78f, 0.82f, 1.00f);
        colors[ImGuiCol_ButtonActive] = ImVec4(0.70f, 0.70f, 0.75f, 1.00f);

        // Header
        colors[ImGuiCol_Header] = ImVec4(0.85f, 0.85f, 0.88f, 1.00f);
        colors[ImGuiCol_HeaderHovered] = ImVec4(0.70f, 0.85f, 0.75f, 1.00f);
        colors[ImGuiCol_HeaderActive] = ImVec4(0.60f, 0.80f, 0.65f, 1.00f);

        // Separator
        colors[ImGuiCol_Separator] = ImVec4(0.78f, 0.78f, 0.80f, 1.00f);
        colors[ImGuiCol_SeparatorHovered] = ImVec4(0.20f, 0.55f, 0.30f, 0.78f);
        colors[ImGuiCol_SeparatorActive] = ImVec4(0.20f, 0.55f, 0.30f, 1.00f);

        // Resize grip
        colors[ImGuiCol_ResizeGrip] = ImVec4(0.20f, 0.55f, 0.30f, 0.20f);
        colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.20f, 0.55f, 0.30f, 0.67f);
        colors[ImGuiCol_ResizeGripActive] = ImVec4(0.20f, 0.55f, 0.30f, 0.95f);

        // Tabs
        colors[ImGuiCol_Tab] = ImVec4(0.88f, 0.88f, 0.90f, 1.00f);
        colors[ImGuiCol_TabHovered] = ImVec4(0.70f, 0.85f, 0.75f, 1.00f);
        colors[ImGuiCol_TabSelected] = ImVec4(0.75f, 0.88f, 0.78f, 1.00f);
        colors[ImGuiCol_TabDimmed] = ImVec4(0.92f, 0.92f, 0.94f, 1.00f);
        colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.82f, 0.90f, 0.84f, 1.00f);

        // Docking
        colors[ImGuiCol_DockingPreview] = ImVec4(0.20f, 0.55f, 0.30f, 0.70f);
        colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.94f, 0.94f, 0.95f, 1.00f);

        // Table
        colors[ImGuiCol_TableHeaderBg] = ImVec4(0.88f, 0.88f, 0.90f, 1.00f);
        colors[ImGuiCol_TableBorderStrong] = ImVec4(0.78f, 0.78f, 0.80f, 1.00f);
        colors[ImGuiCol_TableBorderLight] = ImVec4(0.85f, 0.85f, 0.88f, 1.00f);
        colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
        colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.00f, 0.00f, 0.00f, 0.03f);

        // Text
        colors[ImGuiCol_Text] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
        colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.52f, 1.00f);
        colors[ImGuiCol_TextSelectedBg] = ImVec4(0.20f, 0.55f, 0.30f, 0.35f);

        // Navigation
        colors[ImGuiCol_NavCursor] = ImVec4(0.20f, 0.55f, 0.30f, 1.00f);
        colors[ImGuiCol_NavWindowingHighlight] = ImVec4(0.70f, 0.70f, 0.70f, 0.70f);
        colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.20f, 0.20f, 0.20f, 0.20f);

        // Modal dim
        colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.50f);
    }
} // namespace ui