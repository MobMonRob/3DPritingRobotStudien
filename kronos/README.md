# UR3e 3D Printer — Docker Development Environment

ROS 2 Humble + MoveIt 2 environment for controlling a Universal Robots UR3e arm
as a 3D printer. Reads toolpath files (CSV or G-code) produced by a slicer,
plans collision-aware Cartesian paths, and executes them on a simulated (or real)
UR3e with a print-bed collision object in the MoveIt planning scene.

## Prerequisites

- Docker & Docker Compose v2
- NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (recommended for RViz performance; see below for CPU-only fallback)
- X11 display server (default on most Linux desktops)

## Quick Start

```bash
# Prepare X11 access automatically (Docker or Podman)
./scripts/prepare-x11.sh
# If automatic doesn't work:
xhost +local:docker


# Build and start the container
docker compose up --build -d

# Open a shell inside the container
docker compose exec ur3e-dev bash
```

If you use Podman instead of Docker:

```bash
./scripts/prepare-x11.sh
podman compose up --build -d
podman compose exec ur3e-dev bash
```

When opening this repo as a VS Code Dev Container, X11 setup runs
automatically via `.devcontainer/devcontainer.json` (`initializeCommand`).

### If you do NOT have an NVIDIA GPU

Remove the `deploy.resources` block from `docker-compose.yml` and rely on
software rendering (`LIBGL_ALWAYS_SOFTWARE=1` is already set).

## Usage (inside the container)

All commands below assume you are inside the container
(`docker compose exec ur3e-dev bash`).

### 1. Launch the simulation (UR driver + MoveIt + RViz)

```bash
ros2 launch ur3e_control ur3e_full.launch.py
```

This starts the UR3e fake-hardware driver (with `joint_trajectory_controller`),
waits 5 s, then launches MoveIt 2 + RViz. Wait until you see
**"You can start planning now!"** in the terminal before running the printer.

### 2. Run the 3D printer controller

In a **second** terminal (`docker compose exec ur3e-dev bash`):

```bash
ros2 launch ur3e_control ur3e_printer.launch.py \
    toolpath_file:=/ros2_ws/src/ur3e_control/toolpaths/demo_square.csv
```

The printer node will:

1. Add a print-bed collision object to the planning scene.
2. Load waypoints from the toolpath file.
3. Plan 10 approach paths and pick the shortest, then execute.
4. Descend in a straight Cartesian line to the first waypoint.
5. Execute the remaining toolpath in Cartesian batches (collision-aware).

You should see the robot move in RViz and the terminal log progress like:

```
Batch [0–14] coverage: 100.0 % (15/15 pts)
Progress: 15 / 15 points (100.0 %)
Print complete — 15 / 15 points executed.
```

### 3. Run the TRAC-IK demo controller (optional)

```bash
ros2 launch ur3e_control ur3e_controller.launch.py
```

This runs a standalone demo that solves IK for a few hardcoded waypoints using
TRAC-IK directly, then executes via MoveIt joint planning.

## Toolpath Format

The printer accepts two formats:

**CSV** (default) — one `x, y, z` point per line in metres (base_link frame):

```csv
# comments start with #
0.290, -0.010, 0.112
0.310, -0.010, 0.112
0.310,  0.010, 0.112
```

**G-code** (`.gcode` / `.gco` extension) — standard `G0`/`G1` moves in mm:

```gcode
G1 X290 Y-10 Z112
G1 X310 Y-10 Z112
G1 X310 Y10 Z112
```

Place toolpath files in `src/ur3e_control/toolpaths/` and reference them by
their container path `/ros2_ws/src/ur3e_control/toolpaths/<file>`.

## Development Workflow

Source code lives in `src/ur3e_control/` on the host and is bind-mounted into
the container at `/ros2_ws/src/ur3e_control`. Edit files on the host; rebuild
inside the container:

```bash
cd /ros2_ws
colcon build --packages-select ur3e_control --symlink-install
source install/setup.bash
```

## Connecting to the Real UR3e

Once you've validated your motions in simulation, connect to the physical arm:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=<ROBOT_IP> \
  launch_rviz:=true
```

Then launch MoveIt and the printer controller as above.

## Project Structure

```
.
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
├── README.md
└── src/
  └── ur3e_control/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── kinematics.yaml           # MoveIt kinematics plugin config
    ├── launch/
    │   ├── ur3e_full.launch.py        # UR driver + MoveIt + RViz
    │   ├── ur3e_printer.launch.py     # 3D printer controller
    │   ├── ur3e_controller.launch.py  # TRAC-IK demo
    │   ├── ur3e_gazebo.launch.py
    │   └── ur3e_moveit.launch.py
    ├── toolpaths/
    │   └── demo_square.csv            # sample 3-layer square
    └── src/
      ├── ur3e_printer.cpp           # 3D printer controller
      ├── ur3e_trac_ik_controller.cpp# TRAC-IK IK demo
      └── ur3e_controller.cpp
```
