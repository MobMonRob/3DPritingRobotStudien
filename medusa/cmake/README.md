# CMake Helpers

Project-wide CMake helper utilities.

## 1. Purpose and Responsibility
The `cmake` module contains shared CMake utilities used by the build system.
It does not define targets by itself; it provides reusable functions for consistent logging and configuration.
Within the overall architecture, this folder centralizes build-time helper logic.

## 2. Conceptual Overview
- **Concepts/Patterns**: Centralized helper functions to keep top-level CMake files concise.
- **Design decision: categorized logging**: `medusa_log()` uses `MEDUSA_LOG_CATEGORY` to group messages.

## 3. Directory and File Structure
```
cmake/
└── Utils.cmake  # Global helper functions (logging)
```

## 4. Main Components (Files)
**cmake/Utils.cmake**
- **Responsibility**: Implements `medusa_log()` for standardized, categorized CMake messages.
- **Interfaces**: `medusa_log(msg [level])` with optional CMake log levels.
- **Relation**: Included by the top-level `CMakeLists.txt` and other CMake modules.

## 5. Interfaces and Data Flow
**Inputs**
- `MEDUSA_LOG_CATEGORY` for log tagging.
- Optional log level argument.

**Outputs**
- CMake `message()` output with standardized prefixing and optional color.

**Communication with Other Modules**
- Consumed by `CMakeLists.txt` and external dependency configuration to keep logs uniform.

## 6. Libraries / Dependencies
- No external dependencies; uses standard CMake functions.

## 7. Build Integration
- Included via `include(cmake/Utils.cmake)` in the root `CMakeLists.txt`.

## 8. Usage (for Other Developers)
Minimal usage pattern:

```cmake
set(MEDUSA_LOG_CATEGORY "ROOT")
medusa_log("Configure ${PROJECT_NAME}")
medusa_log("Verbose message", VERBOSE)
```

## 9. Extension and Maintenance
- **Extension points**: Add new helper functions in `cmake/Utils.cmake` as needed.
- **Invariants**:
  - Keep `medusa_log()` behavior stable to preserve log filtering in CI.
- **Limitations**: Current helper set is minimal by design.

## 10. Glossary
- **CMake log level**: The severity passed to `message()` (STATUS, WARNING, FATAL_ERROR, VERBOSE).

