# Settings Data

Runtime settings and UI layout persistence files.

## 1. Purpose and Responsibility
The `data/settings` module stores user- and runtime-level configuration artifacts.
It does not implement settings logic; it only holds persisted files used by the application.
Within the overall architecture, this folder is the default persistence location for UI state and future settings.

## 2. Conceptual Overview
- **Concepts/Patterns**: Persistence of UI state outside the executable to keep sessions reproducible.
- **Design decision: ImGui ini file**: Dear ImGui uses an `.ini` file to persist docking, window positions, and table layout.
- **Design decision: JSON placeholder**: A `settings.json` file is present as a placeholder for structured settings.

## 3. Directory and File Structure
```
data/settings/
├── imgui.ini      # Dear ImGui layout state (windows, docking, tables)
└── settings.json  # Placeholder for persistent application settings (currently empty)
```

## 4. Main Components (Files)
**imgui.ini**
- **Responsibility**: Stores window positions, docking layout, and table state for the UI.
- **Relation**: Written and read by the ImGui layer during startup/shutdown.

**settings.json**
- **Responsibility**: Reserved for structured settings; currently empty.
- **Relation**: Intended to be read/written by the application settings layer.

## 5. Interfaces and Data Flow
**Inputs**
- Runtime UI state produced by Dear ImGui (dock layout, window geometry).

**Outputs**
- Persisted layout state used to restore the UI on the next launch.

**Communication with Other Modules**
- The ImGui layer sets `io.IniFilename` to `MEDUSA_PROJECT_ROOT "/data/settings/imgui.ini"` in `src/ui/im_gui_layer.cpp`.

```mermaid
flowchart LR
    IMGUI[Dear ImGui] -->|write/read| INI[data/settings/imgui.ini]
    APP[App Startup] -->|sets IniFilename| IMGUI
```

## 6. Libraries / Dependencies
- **Dear ImGui**: Uses `imgui.ini` to persist UI layout state.

## 7. Build Integration
- No build integration; files are accessed at runtime via `MEDUSA_PROJECT_ROOT/data/settings`.

## 8. Usage (for Other Developers)
Minimal usage pattern (pseudo-code):

```cpp
ImGuiIO& io = ImGui::GetIO();
io.IniFilename = MEDUSA_PROJECT_ROOT "/data/settings/imgui.ini";
```

## 9. Extension and Maintenance
- **Extension points**: Populate `settings.json` with structured settings as needed.
- **Invariants**:
  - Keep the `imgui.ini` path stable to preserve layout persistence.
  - Ensure new settings remain backward compatible when possible.
- **Limitations**: No schema enforcement for `settings.json` yet.

## 10. Glossary
- **ImGui ini**: Dear ImGui persistence file containing docking and window layout.
- **Persistence**: Saving runtime state for use in subsequent sessions.

