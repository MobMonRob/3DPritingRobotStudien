#ifndef UI_MESH_FILE_BROWSER_H_
#define UI_MESH_FILE_BROWSER_H_

#include <array>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

/**
 * @file file_browser.h
 * @brief Mesh file browser: scans for supported mesh files and renders an ImGui-based browser.
 */

/**
 * @struct MeshFileEntry
 * @brief Represents a discovered mesh file, separating display and absolute path information.
 */
struct MeshFileEntry
{
    /** File name (no directory), e.g. "bunny.stl". */
    std::string file_name;

    /** Folder relative to the scan root (may be empty for root). */
    std::string relative_folder;

    /** Absolute file system path to the mesh file. */
    std::string absolute_path;
};

/**
 * @class MeshFileBrowser
 * @brief Provides a model and ImGui-based UI for browsing and selecting supported mesh files in a directory tree.
 *
 * - Recursively scans a base directory for supported mesh formats (case-insensitive).
 * - Presents a filterable table in ImGui.
 * - Emits the absolute path of the selected entry when the user requests loading.
 */
class MeshFileBrowser
{
public:
    /**
     * @brief Recursively scans baseDirectory and builds the mesh file list.
     *
     * The list is cleared before scanning. If the directory does not exist,
     * an error message is written to status().
     *
     * @param base_directory Directory to scan for supported mesh files.
     */
    void scanDirectory(const std::string& base_directory);

    /**
     * @brief Renders the browser window and handles user interaction.
     *
     * The UI supports two ways to trigger loading:
     * - Double-click on a selected row.
     * - Select a row and click the "Load" button.
     *
     * @param window_title ImGui window title.
     * @param on_load_selected Callback called with the selected entry's absolute path.
     */
    void draw(const char* window_title, const std::function<void(const std::string& absolute_path)>& on_load_selected);

    /**
     * @brief Returns the status line shown at the top of the window.
     * @return Status string (const reference).
     */
    [[nodiscard]] const std::string& status() const
    {
        return mStatusText;
    }

    /**
     * @brief Sets the status line shown at the top of the window.
     * @param text New status message (moved into the browser).
     */
    void setStatus(std::string text)
    {
        mStatusText = std::move(text);
    }

private:
    /** List of discovered mesh files (sorted). */
    std::vector<MeshFileEntry> mFileEntries;
    /** Index into mFileEntries of the currently selected row. */
    int mSelectedEntryIndex{-1};
    /** Raw ImGui input buffer (kept stable across frames). */
    std::array<char, 128> mFilterInputBuffer{};
    /** Current filter query (copied from mFilterInputBuffer). */
    std::string mFilterQuery;
    /** Status text displayed at the top of the browser window. */
    std::string mStatusText;
    /** Remembers the last successfully requested scan directory. */
    std::string mLastScannedDirectory;

    /**
     * @brief Returns true if path ends with a supported mesh extension (case-insensitive).
     * @param path Filesystem path to check.
     * @return True if the extension is supported (case-insensitive).
     */
    static bool hasSupportedExtensionCaseInsensitive(const std::filesystem::path& path);
};

#endif  // UI_MESH_FILE_BROWSER_H_
