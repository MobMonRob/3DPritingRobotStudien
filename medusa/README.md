# Medusa

[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://isocpp.org/std/the-standard)
[![CMake](https://img.shields.io/badge/CMake-3.22+-064F8C.svg)](https://cmake.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![OpenGL](https://img.shields.io/badge/OpenGL-3.2+-5586A4.svg)](https://www.opengl.org/)

A modern C++23 mesh viewer and slicer for 6-axis industrial robot path planning. Import 3D models, visualize them with
OpenGL, slice into layers, and generate motion paths for robotic manufacturing.

![Screenshot](docs/images/screenshot.png)

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
    - [Dependencies](#dependencies)
    - [Build Instructions](#build-instructions)
    - [CMake Options](#cmake-options)
- [Usage](#usage)
    - [Quick Start](#quick-start)
    - [Command-Line Arguments](#command-line-arguments)
    - [Controls](#controls)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
- [API Documentation](#api-documentation)
- [Testing](#testing)
- [Roadmap](#roadmap)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Features

- **Multi-Format Mesh Import**: Load STL and OBJ via Assimp
- **Real-Time 3D Visualization**: OpenGL-based rendering with orbit camera and model rotation
- **Interactive GUI**: Dear ImGui interface with docking support and file browser
- **Cross-Platform**: Runs on Windows, Linux, and macOS
- **Modern C++23**: Clean architecture with RAII, smart resource management, and static analysis
- **Extensible Slicing Pipeline**: Modular design for implementing custom slicing algorithms
- **6-Axis Robot Ready**: Designed for non-planar toolpath generation (in development)

---

## Prerequisites

### Operating System

| OS          | Supported Versions                       |
|-------------|------------------------------------------|
| **Linux**   | Ubuntu 22.04+, Arch, Fedora 38+          |
| **macOS**   | 12.0 Monterey+ (Intel and Apple Silicon) |
| **Windows** | Windows 10/11 with Visual Studio 2022+   |

### Compiler Requirements

| Compiler  | Minimum Version        |
|-----------|------------------------|
| **GCC**   | 13.0+                  |
| **Clang** | 16.0+                  |
| **MSVC**  | 19.35+ (VS 2022 17.5+) |

### Build Tools

- **CMake** 3.22.1 or higher
- **Git** for cloning the repository
- **OpenGL** 3.2+ capable GPU and drivers

### Optional Tools

- **Mold Linker**: Faster linking (auto-detected)
- **CCache**: Build caching (auto-detected)
- **Doxygen**: API documentation generation

---

## Installation

### Dependencies

All dependencies are automatically fetched via CMake FetchContent. No manual installation required.

However, you may need system libraries for OpenGL and windowing:

**Ubuntu/Debian:**

```bash
sudo apt update
sudo apt install build-essential cmake git
sudo apt install libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

**Arch Linux:**

```bash
sudo pacman -S base-devel cmake git mesa libxrandr libxinerama libxcursor libxi
```

**Fedora:**

```bash
sudo dnf install gcc-c++ cmake git mesa-libGL-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel
```

**macOS (Homebrew):**

```bash
brew install cmake git
```

**Windows (vcpkg not required):**

- Install Visual Studio 2022 with "Desktop development with C++" workload
- Install CMake from https://cmake.org/download/

### Build Instructions

```bash
# Clone the repository
git clone https://gitlab.com/your-username/medusa.git
cd medusa

# Create build directory
mkdir build && cd build

# Configure (Release build recommended)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build
cmake --build . --parallel

# Run (Linux/macOS)
./src/app/MedusaApp

# Run (Windows)
.\src\app\Release\MedusaApp.exe
```

### CMake Options

| Option               | Default | Description                                      |
|----------------------|---------|--------------------------------------------------|
| `MEDUSA_BUILD_APP`   | `ON`    | Build the main GUI application                   |
| `MEDUSA_BUILD_TESTS` | `ON`    | Build unit and integration tests                 |
| `CMAKE_BUILD_TYPE`   | `Debug` | Build type: `Debug`, `Release`, `RelWithDebInfo` |

**Example with custom options:**

```bash
cmake -DCMAKE_BUILD_TYPE=Release -DMEDUSA_BUILD_TESTS=OFF ..
```

---

## Usage

### Quick Start

1. **Launch the application:**
   ```bash
   ./build/src/app/MedusaApp
   ```

2. **Load a mesh file:**
    - Use the built-in File Browser panel
    - Double-click a file or select and click "Load"
    - Sample files are included in `data/samplefiles/`

3. **Navigate the 3D view:**
    - Right-click + drag to rotate the camera
    - Scroll wheel to zoom in/out
    - Left-click + drag to rotate the model

### Command-Line Arguments

| Argument     | Description               |
|--------------|---------------------------|
| `[filename]` | Load mesh file on startup |

**Example:**

```bash
./MedusaApp path/to/model.stl
./MedusaApp ../data/samplefiles/cube.obj
```

### Controls

| Input                  | Action                           |
|------------------------|----------------------------------|
| **Right Mouse + Drag** | Orbit camera (azimuth/elevation) |
| **Scroll Wheel**       | Zoom in/out                      |
| **Left Mouse + Drag**  | Rotate model (yaw/pitch)         |
| **Escape**             | Close application                |

---

## Project Structure

```
medusa/
├── CMakeLists.txt          # Root CMake configuration
├── README.md               # This file
├── LICENSE                 # MIT License
├── CHANGELOG               # Version history
├── CONTRIBUTING.md         # Contribution guidelines
├── .clang-format           # Code formatting rules (LLVM-based)
├── .clang-tidy             # Static analysis configuration
├── .gitignore              # Git exclusions
├── .gitlab-ci.yml          # CI/CD pipeline
│
├── cmake/                  # CMake utilities
│   └── Utils.cmake         # Helper macros
│
├── external/               # Third-party dependencies (FetchContent)
│   └── cmake/deps/         # Individual dependency configs
│
├── src/                    # Source code
│   ├── app/                # Application entry point and main loop
│   ├── core/               # Core utilities (Logger, Camera)
│   ├── graphics/           # 3D rendering (Mesh, Shaders, Renderers)
│   ├── controls/           # Input handling (Camera/Model controllers)
│   ├── ui/                 # User interface (ImGui integration)
│   └── slicing/            # Slicing algorithms (in development)
│
├── tests/                  # Test suite
│   ├── unit/               # Unit tests
│   └── integration/        # Integration tests
│
├── docs/                   # Documentation
│   └── doxygen/            # Doxygen configuration
│
└── data/                   # Runtime data
    ├── samplefiles/        # Sample 3D mesh files
    └── settings/           # User settings storage
```

### Module Overview

| Module          | Purpose                                                   |
|-----------------|-----------------------------------------------------------|
| `src/app/`      | Application controller, window management, main loop      |
| `src/core/`     | Logging (spdlog), camera system, shared utilities         |
| `src/graphics/` | OpenGL rendering, mesh loading, shaders, scene management |
| `src/controls/` | Input handling for camera and model manipulation          |
| `src/ui/`       | Dear ImGui integration, file browser, styling             |
| `src/slicing/`  | Mesh slicing and robot path generation (planned)          |

---

## Architecture

Medusa follows a layered MVC-inspired architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                         Application                         │
│                           (App)                             │
├─────────────┬─────────────┬─────────────┬───────────────────┤
│   Controls  │   Graphics  │     UI      │     Slicing       │
│             │             │             │   (planned)       │
├─────────────┴─────────────┴─────────────┴───────────────────┤
│                          Core                               │
│                  (Logger, Camera, Math)                     │
├─────────────────────────────────────────────────────────────┤
│                    External Libraries                       │
│        GLFW | GLAD | GLM | ImGui | Assimp | spdlog          │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

```
File Import (Assimp)
       │
       ▼
   MeshLoader ──► Mesh (GPU Buffers)
       │              │
       ▼              ▼
  BoundingBox    SceneRenderer ──► OpenGL
       │              │
       ▼              ▼
ModelTransform    Framebuffer
       │              │
       ▼              ▼
   Slicing       ImGui Overlay
  (planned)           │
       │              ▼
       ▼           Display
Path Generation
  (planned)
```

### Design Patterns

- **RAII**: Resource management for OpenGL objects (VAO, VBO, shaders)
- **Singleton**: Logger with static initialization
- **MVC Separation**: Clear boundaries between data, rendering, and input
- **Callback Pattern**: File browser uses `std::function` for selection events

---

## API Documentation

The full API documentation is generated with Doxygen and available at `docs/doxygen/html/index.html`.

### Generate Documentation

```bash
cd build
cmake --build . --target doxygen
```

Then open `docs/doxygen/html/index.html` in your browser.

### Key Classes

| Class              | Header                             | Description                  |
|--------------------|------------------------------------|------------------------------|
| `App`              | `src/app/app.h`                    | Main application controller  |
| `Mesh`             | `src/graphics/mesh.h`              | GPU-resident triangle mesh   |
| `MeshLoader`       | `src/graphics/mesh_loader.h`       | Assimp-based file loading    |
| `SceneRenderer`    | `src/graphics/scene_renderer.h`    | 3D scene rendering           |
| `Camera`           | `src/core/camera.h`                | Orbit camera with projection |
| `CameraController` | `src/controls/camera_controller.h` | Camera input handling        |
| `ImGuiLayer`       | `src/ui/im_gui_layer.h`            | Dear ImGui integration       |
| `MeshFileBrowser`  | `src/ui/file_browser.h`            | File browser widget          |

---

## Testing

Medusa uses GoogleTest for unit and integration testing.

### Run All Tests

```bash
cd build
ctest --output-on-failure
```

### Run Specific Test Executable

```bash
./build/tests/unit/MedusaUnitTests
./build/tests/integration/MedusaIntegrationTests
```

### Test Structure

| Directory            | Purpose                                         |
|----------------------|-------------------------------------------------|
| `tests/assets/`      | Sample files for testing (STL, OBJ, etc.)       |
| `tests/unit/`        | Unit tests for individual classes and functions |
| `tests/integration/` | Integration tests for multi-component workflows |

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

### Libraries

- [Assimp](https://github.com/assimp/assimp) - Open Asset Import Library
- [GLM](https://github.com/g-truc/glm) - OpenGL Mathematics
- [GLFW](https://www.glfw.org/) - Multi-platform library for window/input
- [GLAD](https://glad.dav1d.de/) - OpenGL Loader Generator
- [Dear ImGui](https://github.com/ocornut/imgui) - Immediate Mode GUI
- [spdlog](https://github.com/gabime/spdlog) - Fast C++ logging library
- [nlohmann/json](https://github.com/nlohmann/json) - JSON for Modern C++
- [GoogleTest](https://github.com/google/googletest) - C++ testing framework
