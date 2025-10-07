# 3D Printing Robot Studies

An experimental C++ workspace exploring advanced robotic 3D printing techniques. This repository contains two specialized core libraries and their test suites:

- **KRONOS** – Kinematic Robotic Operations & Navigation Orchestration System
  - Kinematics, motion primitives, and low-level control utilities
- **MEDUSA** – Multi-dimensional Extrusion & Dimensional Unified Slicing Algorithm
  - Advanced slicing and path planning for multi-dimensional extrusion

## Repository Structure
- `src/` – Source code for both libraries
  - `kronos/` – Library sources and headers for kinematics/control
  - `medusa/` – Library sources and headers for slicing/path planning
- `tests/` – Test targets for each library
- `docs/` – Project documentation and thesis materials
- `data/` – Sample models and calibration data
- `.clang-tidy` – Clang-Tidy configuration enforcing naming rules
- `.clang-format` – Code formatting configuration

## Prerequisites
- CMake ≥ 4.0.0
- A C++17 compatible compiler (e.g., GCC, Clang, MSVC)
- doxygen (for generating documentation)
- A recent Clang/LLVM toolchain
- Ninja (optional but recommended for faster builds)

## Tests
Run tests via CTest:

```zsh
ctest --test-dir cmake-build-debug --output-on-failure
```

## Linting with Clang-Tidy
This project enforces a Mozilla-like naming scheme through a configured `.clang-tidy` file with the following prefix conventions:

- Parameter names must start with `a` (e.g., `aCount`)
- Global variables with `g` (e.g., `gCache`)
- Non-static data members with `m` (e.g., `mBuffer`)
- Constants with `k` (e.g., `kMaxSize`), including class and member constants

Additionally, we enforce CamelCase for class names and camelBack for method/function names.

## Formatting
A `.clang-format` configuration is available to ensure consistent code style.

## Documentation
See `docs/` for detailed information about:
- Thesis materials and research notes
- Code documentation generated with Doxygen

## License
To be determined.