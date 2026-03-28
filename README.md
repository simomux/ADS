# Autonomous Driving Systems — Course Assignments

Assignments from my Autonomous Driving Systems class.

---

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

---
