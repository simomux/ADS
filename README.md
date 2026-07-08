# Autonomous Driving Systems - Course Assignments

Assignments from my Autonomous Driving Systems class.

## Part 1 - Perception & Tracking (C++ / PCL / ROS 2)

### [Assignment 1 - LiDAR Object Detection via Euclidean Clustering](Part_1/assignment_1/)

3D point cloud segmentation using a custom **KD-tree** and Euclidean clustering algorithm. Detects vehicles and pedestrians from raw LiDAR scans, with voxel downsampling and bounding-box extraction.

**Stack:** C++, PCL, CMake

### [Assignment 2 - Multi-Object Tracking with Kalman Filter](Part_1/assignment_2/)

Full implementation of the **Kalman Filter** predict/update cycle for tracking multiple dynamic objects. Covers covariance tuning, data association between LiDAR clusters and existing tracklets, and track lifecycle management (init, maintenance, deletion).

**Stack:** C++, PCL, CMake

### [Assignment 3 - Map-Based Localization with Particle Filter](Part_1/assignment_3/)

**ROS 2** node implementing a **Particle Filter** for vehicle localization against a prior map using LiDAR landmark observations. Compares random vs. guided initialization, evaluates multiple resampling strategies, and analyzes trajectory estimation error.

**Stack:** C++, ROS 2 (Jazzy), Python (plotting)

---

## Part 2 - Visual Odometry & 3D Reconstruction (Python / OpenCV / GTSAM)

### [Exam Project - Stereo Visual Odometry with Sliding-Window Bundle Adjustment (EuRoC MAV)](Part_2/)

Structure-from-Motion / Visual Odometry pipeline on the **EuRoC MAV** dataset (distorted stereo images from a hexarotor), developed in three incremental stages:

- **`ba_minimum`** - monocular SfM + offline **Bundle Adjustment**: FAST + Lucas-Kanade tracking, **Essential Matrix** bootstrap with RANSAC, incremental **PnP** registration, re-triangulation on inlier drop, batch BA with **GTSAM**
- **`ba_medium`** - **stereo setup**: points tracked both in time (cam0→cam0) and across cameras (cam0→cam1, LK with forward-backward check). The known 11 cm baseline makes the reconstruction **metric** (no scale ambiguity, no E-matrix bootstrap); cam1 observations enter the factor graph via `body_P_sensor` without extra pose variables
- **`ba_advanced`** - batch BA converted to **Visual Odometry**: BA solved on a **sliding window** of the last N frames, with a strong prior on the oldest pose for inter-window continuity (gauge fixing). Includes a parallel PnP-only pipeline as an honest drift baseline, per-window reprojection-RMS diagnostics, and robustness tests on MH_03/MH_05
- Robust noise modeling (**Cauchy m-estimator**), Levenberg-Marquardt on sparse factor graphs, `.ply` export and 3D visualization with **Open3D**

**Stack:** Python, OpenCV, GTSAM, Open3D

---

## Part 3 - Vehicle Dynamics, Control & Planning (Python)

### [Assignment 1 - Vehicle Modeling and Numerical Simulation](Part_3/Assignment_1/)

Three progressively complex vehicle models: **kinematic bicycle**, **linear single-track**, and **nonlinear single-track** with **Pacejka Magic Formula** tyre forces. Compares Euler vs. RK4 numerical integration at different speeds and steering inputs to highlight nonlinear slip angle dynamics.

**Stack:** Python, NumPy, Matplotlib

### [Assignment 2 - Longitudinal & Lateral Control](Part_3/Assignment_2/)

Implements and benchmarks three lateral controllers on a reference oval path at 10–25 m/s:

- **PID** for longitudinal velocity regulation
- **Pure Pursuit** (geometric look-ahead)
- **Stanley** (heading + cross-track error, with 1/v attenuation analysis)
- **MPC** with kinematic bicycle prediction horizon (CasADi optimizer)

Includes analysis of model-mismatch degradation at high speed and lateral-longitudinal coupling effects.

**Stack:** Python, NumPy, CasADi, Matplotlib

### [Assignment 3 - Frenet Optimal Trajectory Planning](Part_3/Assignment_3/)

Optimal trajectory generation in **Frenet coordinates** using quintic/quartic polynomial sampling. Evaluates a candidate set of trajectories via a multi-objective cost function (jerk, time, deviation, obstacle proximity) and selects the minimum-cost path with real-time replanning and static obstacle avoidance.

**Stack:** Python, NumPy, Matplotlib

## Running the assignments

All assignments can be launched from the repo root with a single script (MacOS only):

```bash
uv run python run.py
uv run python run.py <1-7>  # Run a specific assignment
```

| # | Assignment |
| - | ---------- |
| 1 | Part 1 - Euclidean Clustering |
| 2 | Part 1 - Kalman Filter Tracking |
| 3 | Part 1 - Particle Filter (ROS 2) |
| 4 | Part 2 - Stereo VO + Sliding-Window BA (Jupyter) |
| 5 | Part 3 - Vehicle Modeling |
| 6 | Part 3 - Longitudinal & Lateral Control |
| 7 | Part 3 - Frenet Planner |

Extra arguments are forwarded to the underlying script:

```bash
uv run python run.py 3 --plot-only         # Part 1 Assignment 3 - skip simulation, open plotter directly
uv run python run.py 3 --overlap           # Part 1 Assignment 3 - overlaid trajectories with RMSE + 90° rotated view
uv run python run.py 3 --plot-only --overlap  # combine both
```

### Prerequisites

#### Why two environments?

This repo uses two separate environment managers, each handling what it does best:

| | uv (root `.venv/` + `Part_2/.venv/`) | conda (`ros2`) |
| - | ------------- | -------------- |
| **Used for** | Part 3, plotter (root) / Part 2 (isolated) | Part 1 - Assignment 3 |
| **Package source** | PyPI | conda-forge / RoboStack |
| **Why** | Fast, reproducible Python env | ROS 2 packages don't exist on PyPI |

Part 2 lives in its **own isolated uv project** (`Part_2/pyproject.toml`, Python 3.10 with
`numpy==1.22.1`) because the `gtsam` pybind bindings require those exact ABI-compatible versions —
see [Part_2/README.md](Part_2/README.md).

[Pixi](https://pixi.sh) would unify the two into a single tool (it handles both PyPI and conda-forge), but uv was chosen here for the Python side to experiment with it.

ROS 2 packages (`ros-humble-*`) are distributed exclusively as conda packages via [RoboStack](https://robostack.github.io). They include compiled C++ middleware, DDS bindings, and message definitions that have no pip equivalent.

RoboStack is also the only viable way to run ROS 2 natively on macOS ARM: the official ROS 2 builds don't ship binaries for Apple Silicon, building from source is fragile due to LLVM/brew incompatibilities, and Docker on Apple Silicon runs x86 images under emulation with significant overhead.

#### Python dependencies (Part 2 & Part 3)

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then:

```bash
uv sync                  # root env: Part 3 + plotter
cd Part_2 && uv sync     # isolated env for Part 2 (Python 3.10, gtsam-compatible pins)
```

The root `.venv/` covers Part 3 and the plotter (`numpy`, `matplotlib`, `casadi`, ...). Part 2 has
its own `.venv/` under `Part_2/` with `numpy==1.22.1`, `opencv-python`, `gtsam`, `open3d`, `jupyter`
pinned to Python 3.10 for ABI compatibility with the `gtsam` pybind bindings. `run.py 4` launches
the notebook through `uv run` from `Part_2/`, so the right env is picked up automatically.

#### Part 1 - Assignment 1 & 2 (C++ / PCL)

Built automatically by the script via CMake. Requires:

- CMake ≥ 3.5
- PCL ≥ 1.2 - install with `brew install pcl`
- Eigen3 - install with `brew install eigen`

#### Part 1 - Assignment 3 (ROS 2 / Particle Filter)

Requires a conda environment named `ros2` set up with [RoboStack](https://robostack.github.io) (ROS 2 Humble on macOS):

```bash
mamba create -n ros2 python=3.11 \
  -c robostack-staging -c conda-forge \
  ros-humble-desktop \
  ros-humble-pcl-conversions \
  ros-humble-pcl-msgs \
  ros-humble-rosbag2-transport \
  ros-humble-rosbag2-storage-default-plugins \
  colcon-common-extensions \
  ceres-solver \
  gflags
```

The script builds the node automatically if not already built and handles the `DYLD_INSERT_LIBRARIES` workaround required on macOS for ROS 2 Python bindings.

---
