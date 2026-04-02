# Autonomous Driving Systems — Course Assignments

Assignments from my Autonomous Driving Systems class.

## Part 1 — Perception & Tracking (C++ / PCL / ROS 2)

### [Assignment 1 — LiDAR Object Detection via Euclidean Clustering](Part_1/assignment_1/)

3D point cloud segmentation using a custom **KD-tree** and Euclidean clustering algorithm. Detects vehicles and pedestrians from raw LiDAR scans, with voxel downsampling and bounding-box extraction.

**Stack:** C++, PCL, CMake

### [Assignment 2 — Multi-Object Tracking with Kalman Filter](Part_1/assignment_2/)

Full implementation of the **Kalman Filter** predict/update cycle for tracking multiple dynamic objects. Covers covariance tuning, data association between LiDAR clusters and existing tracklets, and track lifecycle management (init, maintenance, deletion).

**Stack:** C++, PCL, CMake

### [Assignment 3 — Map-Based Localization with Particle Filter](Part_1/assignment_3/)

**ROS 2** node implementing a **Particle Filter** for vehicle localization against a prior map using LiDAR landmark observations. Compares random vs. guided initialization, evaluates multiple resampling strategies, and analyzes trajectory estimation error.

**Stack:** C++, ROS 2 (Jazzy), Python (plotting)

---

## Part 2 — Visual Odometry & 3D Reconstruction (Python / OpenCV / GTSAM)

### [Assignment — Visual Odometry with Bundle Adjustment (KITTI dataset)](Part_2/)

End-to-end **monocular visual odometry** pipeline on the KITTI dataset:

- **FAST** feature detection + **Lucas-Kanade Optical Flow** for frame-to-frame tracking
- **Essential Matrix** estimation with RANSAC and relative pose recovery
- Incremental camera registration via **PnP + RANSAC** (6-DoF)
- **Bundle Adjustment** with **GTSAM** (Levenberg-Marquardt optimizer, Cauchy robust noise model) to jointly refine all camera poses and 3D landmarks
- 3D point cloud visualization with **Open3D**

**Stack:** Python, OpenCV, GTSAM, Open3D

---

## Part 3 — Vehicle Dynamics, Control & Planning (Python)

### [Assignment 1 — Vehicle Modeling and Numerical Simulation](Part_3/Assignment_1/)

Three progressively complex vehicle models: **kinematic bicycle**, **linear single-track**, and **nonlinear single-track** with **Pacejka Magic Formula** tyre forces. Compares Euler vs. RK4 numerical integration at different speeds and steering inputs to highlight nonlinear slip angle dynamics.

**Stack:** Python, NumPy, Matplotlib

### [Assignment 2 — Longitudinal & Lateral Control](Part_3/Assignment_2/)

Implements and benchmarks three lateral controllers on a reference oval path at 10–25 m/s:

- **PID** for longitudinal velocity regulation
- **Pure Pursuit** (geometric look-ahead)
- **Stanley** (heading + cross-track error, with 1/v attenuation analysis)
- **MPC** with kinematic bicycle prediction horizon (CasADi optimizer)

Includes analysis of model-mismatch degradation at high speed and lateral-longitudinal coupling effects.

**Stack:** Python, NumPy, CasADi, Matplotlib

### [Assignment 3 — Frenet Optimal Trajectory Planning](Part_3/Assignment_3/)

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
| 1 | Part 1 — Euclidean Clustering |
| 2 | Part 1 — Kalman Filter Tracking |
| 3 | Part 1 — Particle Filter (ROS 2) |
| 4 | Part 2 — Visual Odometry (Jupyter) |
| 5 | Part 3 — Vehicle Modeling |
| 6 | Part 3 — Longitudinal & Lateral Control |
| 7 | Part 3 — Frenet Planner |

Extra arguments are forwarded to the underlying script (Part 3 only):

```bash
uv run python run.py 6 --speed 25
```

### Prerequisites

#### Python dependencies (Part 2 & Part 3)

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then from the repo root:

```bash
uv sync
```

This creates a `.venv/` with all required packages (`numpy`, `matplotlib`, `opencv-python`, `gtsam`, `open3d`, `tqdm`, `jupyter`, `casadi`) pinned to Python 3.11.

#### Part 1 — Assignment 1 & 2 (C++ / PCL)

Built automatically by the script via CMake. Requires:

- CMake ≥ 3.5
- PCL ≥ 1.2 — install with `brew install pcl`
- Eigen3 — install with `brew install eigen`

#### Part 1 — Assignment 3 (ROS 2 / Particle Filter)

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
