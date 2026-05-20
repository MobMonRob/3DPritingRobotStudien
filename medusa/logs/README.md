# Logs

Runtime log output for the application.

## 1. Purpose and Responsibility

The `logs` module stores runtime log files produced by the global logging system.
It does not configure logging; it is the default output directory for the logger.
Within the overall architecture, it provides persisted diagnostics for debugging and tracing.

## 2. Conceptual Overview

- **Concepts/Patterns**: Asynchronous logging with separate console and file sinks.
- **Design decision: rotating file sink**: Logs are rotated by size and count to limit disk usage.
- **Design decision: single default file**: The application writes to `medusa.log` by default.

## 3. Directory and File Structure

```
logs/
├── medusa.log
└── medusa.log.*   # Rotated log files (if size/rotation thresholds are reached)
```

## 4. Main Components (Artifacts)

**medusa.log**

- **Responsibility**: Primary runtime log file.
- **Relation**: Written by `Logger::init()` in `src/core/logger.cpp`.

## 5. Interfaces and Data Flow

**Inputs**

- Log events emitted via `MEDUSA_INFO`, `MEDUSA_WARN`, etc.

**Outputs**

- Text log files with timestamps and thread IDs.

**Communication with Other Modules**

- All modules write through the global logger defined in `src/core/logger.h`.

```mermaid
flowchart LR
    APP[Application] --> LOG[Logger]
    LOG --> FILE[logs/medusa.log]
```

## 6. Libraries / Dependencies

- **spdlog**: Provides asynchronous logging and rotating file sink.

## 7. Build Integration

- No build integration; files are created at runtime by the logger.

## 8. Usage (for Other Developers)

Log output is written automatically when the application runs.

## 9. Extension and Maintenance

- **Extension points**: Adjust rotation size/count or log patterns in `Logger::init()`.
- **Invariants**:
    - Log directory must be writable at runtime.
- **Limitations**: Only one log file name (`medusa.log`) is used by default.

## 10. Glossary

- **Rotating sink**: Log sink that rolls files over after size limits.
- **Asynchronous logging**: Logging that happens on a background thread.

