#include "file_browser.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "imgui.h"
#include "logger.h"

namespace
{
    constexpr std::array<std::string_view, 2> kSupportedMeshExtensions{".stl", ".obj"};
}

bool MeshFileBrowser::hasSupportedExtensionCaseInsensitive(const std::filesystem::path& path)
{
    std::string extension = path.extension().string();
    if (extension.empty())
    {
        return false;
    }
    std::ranges::transform(extension, extension.begin(), [](const unsigned char c) { return std::tolower(c); });
    return std::ranges::any_of(kSupportedMeshExtensions,
                               [&extension](const std::string_view kSupported) { return extension == kSupported; });
}

void MeshFileBrowser::scanDirectory(const std::string& baseDirectory)
{
    MEDUSA_INFO("MeshFileBrowser: Scanning directory '{}' for mesh files", baseDirectory);
    mLastScannedDirectory = baseDirectory;
    mFileEntries.clear();
    mSelectedEntryIndex = -1;
    std::error_code filesystemError;
    const auto kBasePath = std::filesystem::path(baseDirectory);
    if (!std::filesystem::exists(kBasePath, filesystemError))
    {
        mStatusText = "Path not found: " + kBasePath.string();
        MEDUSA_WARN("MeshFileBrowser: Directory does not exist: '{}'", kBasePath.string());
        return;
    }
    int meshFileCount = 0;
    for (auto it = std::filesystem::recursive_directory_iterator(kBasePath, filesystemError);
         it != std::filesystem::recursive_directory_iterator(); it.increment(filesystemError))
    {
        if (filesystemError)
        {
            MEDUSA_WARN("MeshFileBrowser: Filesystem iteration error in '{}': {}", kBasePath.string(),
                        filesystemError.message());
            break;
        }
        if (!it->is_regular_file(filesystemError))
            continue;
        const auto& filePath = it->path();
        if (!hasSupportedExtensionCaseInsensitive(filePath))
            continue;
        MeshFileEntry entry;
        entry.absolute_path = filePath.string();
        const auto kRelative = std::filesystem::relative(filePath, kBasePath, filesystemError);
        entry.file_name = filePath.filename().string();
        entry.relative_folder = (kRelative.has_parent_path() && kRelative.parent_path() != ".")
            ? kRelative.parent_path().string()
            : std::string{};
        mFileEntries.emplace_back(std::move(entry));
        ++meshFileCount;
    }
    std::ranges::sort(mFileEntries, [](const MeshFileEntry& a, const MeshFileEntry& b)
    {
        if (a.file_name == b.file_name)
            return a.relative_folder < b.relative_folder;
        return a.file_name < b.file_name;
    });
    MEDUSA_INFO("MeshFileBrowser: Scan complete, {} mesh file(s) found", meshFileCount);
}

void MeshFileBrowser::draw(const char* window_title,
                           const std::function<void(const std::string& absolute_path)>& on_load_selected)
{
    ImGui::SetNextWindowSize(ImVec2(320, 400), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin(window_title))
    {
        ImGui::End();
        return;
    }

    // --- Status Section ---
    if (!mStatusText.empty())
    {
        // Color the status based on content
        const bool kIsError = mStatusText.find("Failed") != std::string::npos ||
            mStatusText.find("not found") != std::string::npos;
        if (kIsError)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.4f, 0.4f, 1.0f));
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 0.5f, 1.0f));
        }
        ImGui::TextUnformatted(mStatusText.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // --- Search and Actions ---
    if (mFilterInputBuffer[0] == '\0' && !mFilterQuery.empty())
    {
        std::snprintf(mFilterInputBuffer.data(), mFilterInputBuffer.size(), "%s", mFilterQuery.c_str());
    }

    // Search box takes most width, Rescan button on the right
    constexpr float kButtonWidth = 70.0f;
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - kButtonWidth - ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::InputTextWithHint("##filter", "Search...", mFilterInputBuffer.data(), mFilterInputBuffer.size()))
    {
        mFilterQuery = mFilterInputBuffer.data();
    }
    ImGui::PopItemWidth();

    ImGui::SameLine();
    if (ImGui::Button("Rescan", ImVec2(kButtonWidth, 0)))
    {
        MEDUSA_DEBUG("MeshFileBrowser: Rescan requested");
        if (!mLastScannedDirectory.empty())
        {
            scanDirectory(mLastScannedDirectory);
        }
        else
        {
            MEDUSA_WARN("MeshFileBrowser: Rescan requested but no previous directory is known");
        }
    }

    ImGui::Spacing();

    // --- File count ---
    int visibleCount = 0;
    const auto kMatchesFilter = [this](const MeshFileEntry& entry) -> bool
    {
        if (mFilterQuery.empty())
        {
            return true;
        }
        return (entry.file_name.find(mFilterQuery) != std::string::npos) ||
            (entry.relative_folder.find(mFilterQuery) != std::string::npos);
    };

    for (const auto& entry : mFileEntries)
    {
        if (kMatchesFilter(entry))
        {
            ++visibleCount;
        }
    }

    ImGui::TextDisabled("%d file(s)", visibleCount);
    ImGui::Separator();

    // --- File Table ---
    const float kAvailableHeight = ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing() - 8.0f;
    const float kTableHeight = std::max(100.0f, kAvailableHeight);

    constexpr ImGuiTableFlags kTableFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
        ImGuiTableFlags_BordersOuter | ImGuiTableFlags_Resizable;

    if (ImGui::BeginTable("files_table", 2, kTableFlags, ImVec2(0, kTableHeight)))
    {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Folder", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();

        for (int i = 0; i < static_cast<int>(mFileEntries.size()); ++i)
        {
            const auto& entry = mFileEntries[i];
            if (!kMatchesFilter(entry))
            {
                continue;
            }

            ImGui::TableNextRow();

            // Name column
            ImGui::TableNextColumn();
            const bool kIsSelected = (i == mSelectedEntryIndex);

            if (ImGui::Selectable(entry.file_name.c_str(), kIsSelected,
                                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick))
            {
                mSelectedEntryIndex = i;

                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                {
                    MEDUSA_INFO("MeshFileBrowser: Load requested via double-click: '{}'", entry.file_name);
                    if (on_load_selected)
                    {
                        on_load_selected(entry.absolute_path);
                    }
                }
            }

            // Folder column
            ImGui::TableNextColumn();
            if (!entry.relative_folder.empty())
            {
                ImGui::TextDisabled("%s", entry.relative_folder.c_str());
            }
        }

        ImGui::EndTable();
    }

    // --- Load Button ---
    ImGui::Spacing();
    const bool kHasSelection = mSelectedEntryIndex >= 0 &&
        mSelectedEntryIndex < static_cast<int>(mFileEntries.size());

    if (!kHasSelection)
    {
        ImGui::BeginDisabled();
    }

    if (ImGui::Button("Load Selected", ImVec2(-1, 0)))
    {
        if (kHasSelection && on_load_selected)
        {
            const auto& entry = mFileEntries[mSelectedEntryIndex];
            MEDUSA_INFO("MeshFileBrowser: Load requested via button: '{}'", entry.file_name);
            on_load_selected(entry.absolute_path);
        }
    }

    if (!kHasSelection)
    {
        ImGui::EndDisabled();
    }

    ImGui::End();
}