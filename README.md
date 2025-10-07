# 3D Printing Robot Studies

An experimental C++ workspace exploring advanced robotic 3D printing techniques. This repository contains two
specialized core libraries and their test suites:

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

## ROS 2 Docker Development Environment

This repository includes a minimal Docker setup to run ROS 2 Rolling with C++ support, enabling cross-platform
development and testing.

### Docker Setup

1. Build the Docker image:

```bash
docker compose build
```

2. Start the container interactively:

```bash
docker compose run ros
```

3. Your workspace is mounted at `/ros_ws` inside the container, so all changes persist on your host system.

4. ROS 2 environment is automatically sourced, so `ros2` commands and workspace builds work out-of-the-box.

### Create a C++ ROS 2 Package

Inside the container:

```bash
cd /ros_ws/src
ros2 pkg create --build-type ament_cmake my_robot_node --dependencies rclcpp
cd /ros_ws
colcon build --symlink-install
ros2 run my_robot_node <your_node>
```

This allows you to develop and run ROS 2 nodes directly in the container without manual environment setup.

## Tests

Run tests via CTest:

```zsh
ctest --test-dir cmake-build-debug --output-on-failure
```

## Linting with Clang-Tidy

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
