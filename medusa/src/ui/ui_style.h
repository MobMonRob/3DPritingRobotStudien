/**
 * @file ui_style.h
 * @brief Custom ImGui styling for Medusa.
 */

#ifndef MEDUSA_SRC_UI_UI_STYLE_H_
#define MEDUSA_SRC_UI_UI_STYLE_H_

namespace ui
{
    /**
 * @brief Applies custom Medusa styling to ImGui.
 *
 * Call this after ImGui context creation to apply consistent styling.
 */
    void applyMedusaStyle();

    /**
 * @brief Applies dark theme.
 */
    void applyDarkTheme();

    /**
 * @brief Applies light theme.
 */
    void applyLightTheme();
} // namespace ui

#endif // MEDUSA_SRC_UI_UI_STYLE_H_